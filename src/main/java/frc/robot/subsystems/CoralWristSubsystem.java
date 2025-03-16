package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CoralEndEffectorConstants;

public class CoralWristSubsystem implements Subsystem{

    TalonFX coralEndEffectorWrist = new TalonFX(Constants.CoralEndEffectorConstants.WristId, Constants.CANivoreName);

    DCMotorSim wristSim;

    DutyCycleOut manualWristRequest = new DutyCycleOut(0);
    PositionVoltage automaticAngleRequest = new PositionVoltage(0);

    public CoralWristSubsystem() {
        if(RobotBase.isSimulation()) {
            DCMotor wristMotors = DCMotor.getKrakenX60(1);
            wristSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(wristMotors, 0.1, 25), wristMotors);
        }

        coralEndEffectorWrist.getConfigurator().apply(
            new TalonFXConfiguration().withSlot0(
                new Slot0Configs().withKP(3)
            )
        );
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState wristSimState = coralEndEffectorWrist.getSimState();

        wristSim.setInput(wristSimState.getMotorVoltage());

        wristSim.update(0.02);
        
        wristSimState.setRawRotorPosition(wristSim.getAngularPosition());
        wristSimState.setRotorVelocity(wristSim.getAngularVelocity());
    }

    public enum WristAngles{
        Collect(Rotation.of(0)),
        L1(Rotation.of(4)),
        L2OrL3(Rotation.of(3)),
        L4(Rotation.of(3)),
        ;

        final Angle Degree;
        private WristAngles(Angle degree) {
            this.Degree = degree;
        }
    }

    private void manualWrist(double output) {
        coralEndEffectorWrist.setControl(manualWristRequest.withOutput(output));
    }
    
    private void goToAngle(WristAngles angleToGoTo){
        coralEndEffectorWrist.setControl(automaticAngleRequest.withPosition(angleToGoTo.Degree));
    }

    public Command goToAngleCommand(Supplier<WristAngles> endEffectorAngleProvider) {
        return run(()-> {
            goToAngle(endEffectorAngleProvider.get());
        });
    }

    public Command manualWrist(DoubleSupplier outputSupplier) {
        return run(()-> {
            manualWrist(outputSupplier.getAsDouble());
        });
    }

    public Command zeroWristCommand() {
        return run(()->manualWrist(-0.1)).until(
            new Trigger(()->coralEndEffectorWrist.getVelocity().getValueAsDouble() < 2 && coralEndEffectorWrist.getStatorCurrent().getValueAsDouble() > 5).debounce(0.2))
            .andThen(runOnce(()->coralEndEffectorWrist.setPosition(0)));
    }

    public Command zeroWristInPlaceCommand() {
        return runOnce(()->coralEndEffectorWrist.setPosition(0));
    }
}
