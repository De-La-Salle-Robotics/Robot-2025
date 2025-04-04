package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ElevatorSubsystem implements Subsystem {

    private double elevatorUpOutput = 0.3;
    private double elevatordownOutput = -0.3;
    private double elevatorStopOutput = 0.0;

    TalonFX elevatorLeft = new TalonFX(Constants.ElevatorConstants.LeftId, Constants.CANivoreName);
    TalonFX elevatorRight = new TalonFX(Constants.ElevatorConstants.RightId, Constants.CANivoreName);

    StatusSignal<Angle> leftPosition = elevatorLeft.getPosition();
    StatusSignal<Angle> rightPosition = elevatorRight.getPosition();
    StatusSignal<Double> leftOutput = elevatorLeft.getDutyCycle();
    StatusSignal<Double> rightOutput = elevatorRight.getDutyCycle();

    DCMotorSim elevatorSim;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    MotionMagicVoltage automaticHeightRequest = new MotionMagicVoltage(0);

    DoublePublisher leftHeightPublisher = Constants.ElevatorConstants.ElevatorTable.getDoubleTopic("Left Height").publish();
    DoublePublisher rigthHeightPublisher = Constants.ElevatorConstants.ElevatorTable.getDoubleTopic("Right Height").publish();
    DoublePublisher leftOutputPublisher = Constants.ElevatorConstants.ElevatorTable.getDoubleTopic("Left Output").publish();
    DoublePublisher rightOutputPublisher = Constants.ElevatorConstants.ElevatorTable.getDoubleTopic("Right Output").publish();

    public ElevatorSubsystem() {
        if (RobotBase.isSimulation()) {
            DCMotor elevatorMotor = DCMotor.getKrakenX60(2);

            elevatorSim = new DCMotorSim(LinearSystemId.createElevatorSystem(elevatorMotor, 10, 0.3, 10), elevatorMotor);
        }

        elevatorLeft.getConfigurator().apply(
            new TalonFXConfiguration().withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake))
                                    .withSlot0(new Slot0Configs().withKP(10))
                                    .withMotionMagic(new MotionMagicConfigs().withMotionMagicCruiseVelocity(60)
                                                                            .withMotionMagicAcceleration(300)));
        elevatorRight.getConfigurator().apply(
            new TalonFXConfiguration().withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake)));

        elevatorRight.setControl(new Follower(elevatorLeft.getDeviceID(), true));
    }
    @Override
    public void simulationPeriodic() {
        TalonFXSimState elevatorSimState = elevatorLeft.getSimState();
        TalonFXSimState followSimState = elevatorRight.getSimState();

        elevatorSim.setInputVoltage((elevatorSimState.getMotorVoltage() + followSimState.getMotorVoltage()) / 2);

        elevatorSim.update(0.02);

        elevatorSimState.setRawRotorPosition(elevatorSim.getAngularPosition());
        elevatorSimState.setRotorVelocity(elevatorSim.getAngularVelocity());
        followSimState.setRawRotorPosition(elevatorSim.getAngularPosition());
        followSimState.setRotorVelocity(elevatorSim.getAngularVelocity());
    }

    @Override
    public void periodic() {
        leftHeightPublisher.accept(leftPosition.refresh().getValueAsDouble());
        rigthHeightPublisher.accept(rightPosition.refresh().getValueAsDouble());

        leftOutputPublisher.accept(leftOutput.refresh().getValueAsDouble());
        rightOutputPublisher.accept(rightOutput.refresh().getValueAsDouble());
    }

    public enum ElevatorHeights{
        Stowed(Rotations.of(0)),
        ReadyToCollect(Rotations.of(10)),
        L1(Rotations.of(12)),
        L2(Rotations.of(22)),
        L3(Rotations.of(33.6)),
        L4(Rotations.of(20)),
        ;

        final Angle Height;
        private ElevatorHeights(Angle height) {
            this.Height = height;
        }
    }

    private void manualDriveTalons(double output) {
        elevatorLeft.setControl(manualControlRequest.withOutput(output));
    }

    public Trigger isNearSetpoint(ElevatorHeights target) {
        return new Trigger(() -> (elevatorLeft.getPosition().getValue().minus(target.Height).abs(Rotations) < 1));
    }

    public Command manualElevatorCommand(DoubleSupplier elevatorInput) {
        return run(()-> {
            manualDriveTalons(elevatorInput.getAsDouble());
        });
    }

    private void goToHeight(ElevatorHeights heightToGoTo){
        elevatorLeft.setControl(automaticHeightRequest.withPosition(heightToGoTo.Height));
    }

    public Command goToHeightCommand(Supplier<ElevatorHeights> elevatorHeightProvider) {
        return run(()-> {
            goToHeight(elevatorHeightProvider.get());
        });
    }

    public Command elevatorUpCommand() {
        return run(()->{
            manualDriveTalons(elevatorUpOutput);
        }).finallyDo(()->{manualDriveTalons(0);});
    }

    public Command elevatorDownCommand() {
        return run(()->{
            manualDriveTalons(elevatordownOutput);
        }).finallyDo(()->{manualDriveTalons(0);});
    }

    public Command elevatorStopCommand() {
        return run(()->{
            manualDriveTalons(elevatorStopOutput);
        });
    }

    public Command zeroInPlaceCommand() {
        return runOnce(()->elevatorLeft.setPosition(0));
    }
}
