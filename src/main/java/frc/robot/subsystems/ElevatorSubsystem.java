package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ElevatorSubsystem implements Subsystem {
    TalonFX elevatorLeft;
    TalonFX elevatorRight;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    MotionMagicVoltage automaticHeightRequest = new MotionMagicVoltage(0);

    public ElevatorSubsystem(){
        elevatorRight.setControl(new Follower(elevatorLeft.getDeviceID(), false));
    }

    public enum ElevatorHeights{
        Stowed(Rotations.of(0)),
        L1(Rotations.of(20)),
        L2(Rotations.of(40)),
        L3(Rotations.of(60)),
        L4(Rotations.of(80)),
        ;

        final Angle Height;
        private ElevatorHeights(Angle height) {
            this.Height = height;
        }
    }

    private void manualDriveTalons(double output) {
        elevatorLeft.setControl(manualControlRequest.withOutput(output));
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
        return run(()-> {goToHeight(elevatorHeightProvider.get());
            
        });
    }
}
