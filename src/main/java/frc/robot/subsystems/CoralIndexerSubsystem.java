package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralEndEffectorSubsystem.WristAngles;

public class CoralIndexerSubsystem implements Subsystem{
    private double FlippersZeroAngle = 0;
    private double FlippersOpenAngle = 45;
    private double FlippersClosedAngle = 0;
    private double OpenFlippersOutput = 0.5;
    private double CloseFlippersOutput = -0.5;
    private double SlowCloseFlippersOutput = -0.1;
    private double FlippersStopVelocity = -0.01;

    CommonTalon coralGroundIntake;
    CommonTalon coralFlippers;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    MotionMagicVoltage automaticAngleRequest = new MotionMagicVoltage(0);

    public enum CoralGroundIntakeAngles{
        Stowed(Degrees.of(0)),
        ReadyToGrab(Degrees.of(60)),
        Ground(Degrees.of(90)),
        ;

        final Angle Degree;
        private CoralGroundIntakeAngles(Angle degree) {
            this.Degree = degree;
        }
    }

    private void goToAngle(CoralGroundIntakeAngles angleToGoTo){
        coralGroundIntake.setControl(automaticAngleRequest.withPosition(angleToGoTo.Degree));
    }

    public Command goToAngleCommand(Supplier<CoralGroundIntakeAngles> CoralGroundIntakeAngleProvider) {
        return run(()-> {goToAngle(CoralGroundIntakeAngleProvider.get());
            
        });
    }

    private void manualDriveCoralFlippers(double output) {
        coralFlippers.setControl(manualControlRequest.withOutput(output));
    } 

    public Command openCoralFlippers() {
        return run(()-> {
            manualDriveCoralFlippers(OpenFlippersOutput);
        })
        .until(flippersOpenTrigger());
    }

    public Command closeCoralFlippers() {
        return run(()-> {
            manualDriveCoralFlippers(CloseFlippersOutput);
        })
        .until(flippersClosedTrigger());
    }

    public double getFlippersAngle() {
        return coralFlippers.getPosition().getValueAsDouble()-FlippersZeroAngle;
    }

    public void zeroFlippers(){
        FlippersZeroAngle = coralFlippers.getPosition().getValueAsDouble();
    }

    public Command manualZeroFlippers() {
        return run(()->manualDriveCoralFlippers(SlowCloseFlippersOutput)).until(new Trigger(()->{
            return coralFlippers.getVelocity().getValueAsDouble()>FlippersStopVelocity;
        }).debounce(1)).andThen(
        runOnce(this::zeroFlippers)); 
    }

    public Trigger flippersClosedTrigger() {
        return new Trigger(()->{
            return getFlippersAngle() <=FlippersClosedAngle;
        });
    }

    public Trigger flippersOpenTrigger() {
        return new Trigger(()->{
            return getFlippersAngle() >=FlippersOpenAngle;
        });
    }

}
