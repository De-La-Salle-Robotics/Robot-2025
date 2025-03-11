// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.security.GuardedObject;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralEndEffectorSubsystem;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralEndEffectorSubsystem.WristAngles;
import frc.robot.subsystems.CoralIndexerSubsystem.CoralGroundIntakeAngles;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeights;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralIndexerSubsystem indexer = new CoralIndexerSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final CoralEndEffectorSubsystem endEffector = new CoralEndEffectorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverJoystick.y().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L4).alongWith(endEffector.goToAngleCommand(()->WristAngles.L4)));
        driverJoystick.x().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L3).alongWith(endEffector.goToAngleCommand(()->WristAngles.L2OrL3)));
        driverJoystick.b().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L2).alongWith(endEffector.goToAngleCommand(()->WristAngles.L2OrL3)));
        driverJoystick.a().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L1).alongWith(endEffector.goToAngleCommand(()->WristAngles.L1)));
        driverJoystick.leftBumper().onTrue(indexer.goToAngleCommand(()->CoralGroundIntakeAngles.Ground));
        driverJoystick.leftTrigger().onTrue(indexer.goToAngleCommand(()->CoralGroundIntakeAngles.Stowed));
        driverJoystick.rightBumper().onTrue(endEffector.suckCommand());
        driverJoystick.rightTrigger().onTrue(endEffector.spitCommand());


        operatorJoystick.rightBumper().onTrue(indexer.openCoralFlippers());
        operatorJoystick.rightTrigger().onTrue(indexer.closeCoralFlippers());
        operatorJoystick.start().onTrue(indexer.manualZeroFlippers());
        operatorJoystick.b().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.ReadyToCollect));
        operatorJoystick.x().onTrue(indexer.goToAngleCommand(()->CoralGroundIntakeAngles.ReadyToGrab));
        operatorJoystick.a().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.Stowed).alongWith(endEffector.suckCommand()));

        // reset the field-centric heading on left bumper press
        operatorJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
