package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeWristAngles;
import frc.utils.Trigger;

public class AlgaeSubsystem implements Subsystem {
    private double slurpOutput = 0.3;
    private double blowOutput = -0.3;
    private double AlgaeStopVelocity = 0.1;
    private double AlgaeStopOutput = 0;

    TalonFX algaeWrist = new TalonFX(Constants.AlgaeConstants.algaeWristId, Constants.CANivoreName);
    TalonFX algaeGrab = new TalonFX(Constants.AlgaeConstants.algaeGrabId, Constants.CANivoreName);

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);
    DutyCycleOut manualWristRequest = new DutyCycleOut(0);
    PositionVoltage automaticAngleRequest = new PositionVoltage(0);

    public Command suckUntilHaveAlgaeCommand() {
        return run(() -> manualDrivealgaeGrab(slurpOutput)).until(new Trigger(() -> {
            return algaeGrab.getVelocity().getValueAsDouble() < AlgaeStopVelocity;
        }).debounce(1, DebounceType.kRising).asRisingEdge()).andThen(
                run(() -> manualDrivealgaeGrab(AlgaeStopOutput)));
    }

    public Command slurpCommand() {
        return run(() -> {
            manualDrivealgaeGrab(slurpOutput);
        });
    }

    public Command blowCommand() {
        return run(() -> {
            manualDrivealgaeGrab(blowOutput);
        });
    }

    private void manualDrivealgaeGrab(double output) {
        algaeGrab.setControl(manualControlRequest.withOutput(output));
    }

    public Command manualCoralEndEffectorCommand(DoubleSupplier algaeGrab) {
        return run(() -> {
            manualDrivealgaeGrab(algaeGrab.getAsDouble());
        });
    }

    public enum AlgaeWristAngles {
        GroundCollect(Degrees.of(1)),
        ReefCollect(Degrees.of(2)),
        Processor(Degrees.of(3)),
        Net(Degrees.of(45)),
        StowedAlgae(Degrees.of(90)),
        ;

        final Angle Degree;

        private AlgaeWristAngles(Angle degree) {
            this.Degree = degree;
        }
    }

    private void manualWrist(double output) {
        algaeWrist.setControl(manualWristRequest.withOutput(output));
    }

    public Command manualWrist(DoubleSupplier outputSupplier) {
        return run(() -> {
            manualWrist(outputSupplier.getAsDouble());
        });
    }

    private void goToAngle(AlgaeWristAngles angleToGoTo) {
        algaeWrist.setControl(automaticAngleRequest.withPosition(angleToGoTo.Degree));
    }

    public Command goToAngleCommand(Supplier<AlgaeWristAngles> endEffectorAngleProvider) {
        return run(() -> {
            goToAngle(endEffectorAngleProvider.get());
        });
    }

    public Command zeroWristCommand() {
        return run(() -> manualWrist(-0.1)).until(
                new Trigger(() -> algaeWrist.getVelocity().getValueAsDouble() < 2
                        && algaeWrist.getStatorCurrent().getValueAsDouble() > 5).debounce(0.2))
                .andThen(runOnce(() -> algaeWrist.setPosition(0)));
    }

    public Command zeroWristInPlaceCommand() {
        return runOnce(() -> algaeWrist.setPosition(0));
    }

    public Command neutralAlgaeOutputCommand() {
        return run(()->{
            manualDriveTalons(0);
        });
    }

    private void manualDriveTalons(double output) {
        algaeWrist.setControl(manualControlRequest.withOutput(output));
    }
}