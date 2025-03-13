package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.Trigger;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeights;

public class CoralEndEffectorSubsystem implements Subsystem{
    private double SuckOutput = 0.3;
    private double SpitOutput = -0.3;
    private double StopOutput = 0;
    private double EndEffectorRollersStopVelocity = 0.1;

    TalonFX coralEndEffectorRollers = new TalonFX(13, "canivore");
    TalonFX coralEndEffectorWrist = new TalonFX(14, "canivore");

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    MotionMagicVoltage automaticAngleRequest = new MotionMagicVoltage(0);

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
