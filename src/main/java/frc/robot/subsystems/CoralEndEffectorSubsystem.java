package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class CoralEndEffectorSubsystem implements Subsystem{
    private double SuckOutput = 0.3;
    private double SpitOutput = -0.3;
    private double StopOutput = 0;
    private double EndEffectorRollersStopVelocity = 0.1;

    TalonFX coralEndEffectorRollers;
    TalonFX coralEndEffectorWrist;

    DCMotorSim rollerSim;
    DCMotorSim wristSim;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    MotionMagicVoltage automaticAngleRequest = new MotionMagicVoltage(0);

    public CoralEndEffectorSubsystem() {
        coralEndEffectorRollers = new TalonFX(CoralEndEffectorConstants.RollerId);
        coralEndEffectorWrist = new TalonFX(Constants.CoralEndEffectorConstants.WristId, Constants.CANivoreName);

        if(RobotBase.isSimulation()) {
            DCMotor rollerMotors = DCMotor.getKrakenX60(1);
            DCMotor wristMotors = DCMotor.getKrakenX60(1);
            rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerMotors, 0.1, 1), rollerMotors);
            wristSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(wristMotors, 0.1, 25), wristMotors);
        }
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState rollerSimState = coralEndEffectorRollers.getSimState();
        TalonFXSimState wristSimState = coralEndEffectorWrist.getSimState();

        rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());
        wristSim.setInput(wristSimState.getMotorVoltage());

        rollerSim.update(0.02);
        wristSim.update(0.02);

        rollerSimState.setRawRotorPosition(rollerSim.getAngularPosition());
        rollerSimState.setRotorVelocity(rollerSim.getAngularVelocity());
        
        wristSimState.setRawRotorPosition(wristSim.getAngularPosition());
        wristSimState.setRotorVelocity(wristSim.getAngularVelocity());
    }

    public enum WristAngles{
        Collect(Degrees.of(0)),
        L1(Degrees.of(90)),
        L2OrL3(Degrees.of(35)),
        L4(Degrees.of(20)),
        ;

        final Angle Degree;
        private WristAngles(Angle degree) {
            this.Degree = degree;
        }
    }

    private void manualDriveCoralEndEffectorRollers(double output) {
        coralEndEffectorRollers.setControl(manualControlRequest.withOutput(output));
    }
    
    public Command manualCoralEndEffectorCommand(DoubleSupplier CoralEndEffectorInput) {
        return run(()-> {
            manualDriveCoralEndEffectorRollers(CoralEndEffectorInput.getAsDouble());
        });
    }
    
    private void goToAngle(WristAngles angleToGoTo){
        coralEndEffectorWrist.setControl(automaticAngleRequest.withPosition(angleToGoTo.Degree));
    }

    public Command goToAngleCommand(Supplier<WristAngles> endEffectorAngleProvider) {
        return run(()-> {goToAngle(endEffectorAngleProvider.get());
            
        });
    }

    public Command suckCommand() {
        return run(()->{
            manualDriveCoralEndEffectorRollers(SuckOutput);
        });
    }

    public Command spitCommand() {
        return run(()->{
            manualDriveCoralEndEffectorRollers(SpitOutput);
        });
    }

    public Command suckUntilHaveCoralCommand() {
        return run(()->manualDriveCoralEndEffectorRollers(SuckOutput)).until(new Trigger(()->{
            return coralEndEffectorRollers.getVelocity().getValueAsDouble()<EndEffectorRollersStopVelocity;
        }).debounce(1, DebounceType.kRising).asRisingEdge()).andThen(
        run(()->manualDriveCoralEndEffectorRollers(StopOutput))); 
    }

}
