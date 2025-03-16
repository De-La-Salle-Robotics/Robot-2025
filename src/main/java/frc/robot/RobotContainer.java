// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollerSubsystem;
import frc.robot.subsystems.CoralWristSubsystem;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralIndexerSubsystem.CoralGroundIntakeAngles;
import frc.robot.subsystems.CoralWristSubsystem.WristAngles;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeights;
import frc.utils.Trigger;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralIndexerSubsystem indexer = new CoralIndexerSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final CoralRollerSubsystem endEffector = new CoralRollerSubsystem();
    public final CoralWristSubsystem wrist = new CoralWristSubsystem();
    public final ClimbSubsystem climb = new ClimbSubsystem();
    
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {

        NamedCommands.registerCommand("ZeroFlippers",(indexer.manualZeroFlippers()));
        NamedCommands.registerCommand("GoToL4",(elevator.goToHeightCommand(()->ElevatorHeights.L4).alongWith(wrist.goToAngleCommand(()->WristAngles.L4))));
        NamedCommands.registerCommand("GoToL3",(elevator.goToHeightCommand(()->ElevatorHeights.L3).alongWith(wrist.goToAngleCommand(()->WristAngles.L2OrL3))));
        NamedCommands.registerCommand("GoToL2",(elevator.goToHeightCommand(()->ElevatorHeights.L2).alongWith(wrist.goToAngleCommand(()->WristAngles.L2OrL3))));
        NamedCommands.registerCommand("GoToL1",(wrist.goToAngleCommand(()->WristAngles.L1)));
        NamedCommands.registerCommand("Spit",(endEffector.spitCommand()));
        NamedCommands.registerCommand("closeFlippers", indexer.closeCoralFlippers());
        NamedCommands.registerCommand("OpenFlippers", indexer.openCoralFlippers());
        NamedCommands.registerCommand("SuckUntilHaveCoral",(elevator.goToHeightCommand(()->ElevatorHeights.Stowed).alongWith(endEffector.suckUntilHaveCoralCommand())));
        NamedCommands.registerCommand("ReadyToCollect", (elevator.goToHeightCommand(()->ElevatorHeights.ReadyToCollect)));
        NamedCommands.registerCommand("GroundIntakeUp", (indexer.goToAngleCommand(()->CoralGroundIntakeAngles.Stowed)));
        NamedCommands.registerCommand("ZeroWristInPlace", wrist.zeroWristInPlaceCommand());

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
        
        indexer.setDefaultCommand(indexer.run(()->{}));
        elevator.setDefaultCommand(elevator.run(()->{}));
        endEffector.setDefaultCommand(endEffector.manualCoralEndEffectorCommand(()->0));
        wrist.setDefaultCommand(wrist.manualWrist(()->-operatorJoystick.getLeftY() * 0.1));
        climb.setDefaultCommand(climb.neutralOutputCommand());

        driverJoystick.y().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L4).alongWith(wrist.goToAngleCommand(()->WristAngles.L4)));
        driverJoystick.x().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L3).alongWith(wrist.goToAngleCommand(()->WristAngles.L2OrL3)));
        driverJoystick.b().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L2).alongWith(wrist.goToAngleCommand(()->WristAngles.L2OrL3)));
        driverJoystick.a().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.L1).alongWith(wrist.goToAngleCommand(()->WristAngles.L1)).until(()->Math.abs(operatorJoystick.getLeftY()) > 0.2));
        
        driverJoystick.leftBumper().onTrue(indexer.goToAngleCommand(()->CoralGroundIntakeAngles.Ground));
        driverJoystick.leftTrigger().onTrue(indexer.goToAngleCommand(()->CoralGroundIntakeAngles.Stowed));
        driverJoystick.rightBumper().whileTrue(endEffector.suckCommand());
        driverJoystick.rightTrigger().whileTrue(endEffector.spitCommand());

        operatorJoystick.rightBumper().onTrue(indexer.openCoralFlippers());
        operatorJoystick.rightTrigger().onTrue(indexer.closeCoralFlippers());
        operatorJoystick.start().onTrue(indexer.manualZeroFlippers());
        operatorJoystick.b().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.ReadyToCollect));
        operatorJoystick.x().onTrue(indexer.goToAngleCommand(()->CoralGroundIntakeAngles.ReadyToGrab));
        operatorJoystick.a().onTrue(elevator.goToHeightCommand(()->ElevatorHeights.Stowed).alongWith(endEffector.suckUntilHaveCoralCommand()));
        operatorJoystick.y().onTrue(wrist.zeroWristInPlaceCommand());
        operatorJoystick.leftTrigger().whileTrue(elevator.elevatorDownCommand());
        operatorJoystick.leftBumper().whileTrue(elevator.elevatorUpCommand());
        operatorJoystick.povUp().whileTrue(climb.peckCommand());
        operatorJoystick.povDown().whileTrue(climb.climbCommand());
        operatorJoystick.povRight().whileTrue(climb.stopClimbCommand());


        // reset the field-centric heading on left bumper press
        driverJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // new Trigger(DriverStation::isDisabled).debounce(5).onTrue(new InstantCommand(()->climb.setCoastMode()).ignoringDisable(true));
        // new Trigger(DriverStation::isEnabled).onTrue(new InstantCommand(climb::setBrakeMode).alongWith(new InstantCommand(climb::initStopServo)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
