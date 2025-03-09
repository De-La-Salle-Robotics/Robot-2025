package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimbSubsystem implements Subsystem {
    private double peckOutput = 0.3;
    private double climbOutput = -0.3;

    TalonFX climb;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);

    private void manualDriveClimb(double output) {
        climb.setControl(manualControlRequest.withOutput(output));
    }

    private void manualDriveTalons(double output) {
        climb.setControl(manualControlRequest.withOutput(output));
    }

    public Command manualClimbCommand(DoubleSupplier climbInput) {
        return run(()-> {
            manualDriveTalons(climbInput.getAsDouble());
        });
    }

    public Command peckCommand() {
        return run(()->{
            manualDriveTalons(peckOutput);
        });
    }

    public Command climbCommand() {
        return run(()->{
            manualDriveTalons(climbOutput);
        });
    }
}