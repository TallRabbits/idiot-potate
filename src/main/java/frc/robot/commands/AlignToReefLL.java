// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefLL extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  /** Creates a new AlignToReefLL. */
  public AlignToReefLL(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP_aim = .025;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-swerve") * kP_aim;

    // convert to meters per second for our drive method
    targetingAngularVelocity *= RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;
    SmartDashboard.putNumber("Auto Align AV", targetingAngularVelocity);

    double kP_range = .03;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-swerve") * kP_range;
    targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    targetingForwardSpeed *= -1.0;
    SmartDashboard.putNumber("Auto Align X", targetingForwardSpeed);

    m_drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(targetingForwardSpeed).withRotationalRate(targetingAngularVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}