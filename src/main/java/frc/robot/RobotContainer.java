// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotStates;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.Dealgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.RobotToState;
import frc.robot.commands.ScoreCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.pooper.Pooper;
import frc.robot.subsystems.elevator.Elevator;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController dev = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Pooper pooper = new Pooper();
    

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("L3", new RobotToState(elevator, pooper, RobotStates.L3));
        NamedCommands.registerCommand("L4", new RobotToState(elevator, pooper, RobotStates.L4));
        NamedCommands.registerCommand("spitCoral", new ScoreCoral(elevator, pooper));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Zero Elevator", Commands.runOnce(() -> elevator.resetElevator()));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                driveRobotCentric.withVelocityX(-joystick.getLeftY() * MaxSpeed/4) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed/4) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );

        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // CORAL INTAKE AND SCORING //
        joystick.rightTrigger(0.5)
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.CORAL_STATION)
            .alongWith(new IntakeCoral(pooper))
        );

        joystick.povUp()
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.L4)
            //.alongWith(new AlignToReef(false, drivetrain))
        );

        joystick.povRight()
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.L3)
            //.alongWith(new AlignToReef(false, drivetrain))
        );

        joystick.povLeft()
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.L2)
            //.alongWith(new AlignToReef(false, drivetrain))
        );

        joystick.povDown()
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.L1)
            //.alongWith(new AlignToReef(false, drivetrain))
        );

        joystick.x()
            .whileTrue(new ScoreCoral(elevator, pooper)
        );

        joystick.a()
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.DEALGAE_LOWER)
            .alongWith(new Dealgae(pooper))
        );

        joystick.b()
            .whileTrue(new RobotToState(elevator, pooper, RobotStates.DEALGAE_UPPER)
            .alongWith(new Dealgae(pooper))
        );

        joystick.y()
            .onTrue(pooper.runOnce(() -> pooper.runAlgaeRoller(-7))
        );
        // joystick.leftBumper()
        //     .onTrue(new RobotToState(elevator, pooper, RobotStates.BARGE)
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Start experimental controls
        joystick.button(8).whileTrue(new AlignToReef(true, drivetrain));
        joystick.button(7).whileTrue(new AlignToReef(false, drivetrain));
        joystick.leftTrigger(0.25)
            .whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0.25).withVelocityY(0))
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
