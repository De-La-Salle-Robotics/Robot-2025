package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CoralIndexerSubsystem implements Subsystem{
    CommonTalon coralGroundIntake;
    CommonTalon coralFlipers;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    MotionMagicVoltage automaticHeightRequest = new MotionMagicVoltage(0);

    public enum CoralGroundIntakeAngles{
        Stowed(Degrees.of(0)),
        Ready(Degrees.of(60)),
        Ground(Degrees.of(90)),
        ;

        final Angle Degree;
        private CoralGroundIntakeAngles(Angle degree) {
            this.Degree = degree;
        }
    }
}
