// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  private final PIDController m_xController = new PIDController(5, 0, 0);
  private final PIDController m_yController = new PIDController(5, 0, 0);
  private final PIDController m_rotController = new PIDController(5, 0, 0);

  private final boolean m_isRightPipe;
  private final CommandSwerveDrivetrain m_drivetrain;

  private final Timer m_invalidTagTimer = new Timer();
  private final Timer m_alignedDebounceTimer = new Timer();

  private double tagID = -1;

  /** Creates a new AlignToReef. */
  public AlignToReef(boolean isRightPipe, CommandSwerveDrivetrain drivetrain) {
    m_isRightPipe = isRightPipe;
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_invalidTagTimer.start();
    m_alignedDebounceTimer.start();

    m_xController.setSetpoint(m_isRightPipe ? 13 : -29);
    m_yController.setSetpoint(-8.25);
    m_rotController.setSetpoint(0);

    m_xController.setTolerance(0.1);
    m_yController.setTolerance(0.1);
    m_rotController.setTolerance(1);

    tagID = LimelightHelpers.getFiducialID("limelight-swerve");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-swerve") && LimelightHelpers.getFiducialID("limelight-swerve") == tagID) {
      m_invalidTagTimer.reset();

      double velocityX = m_xController.calculate(LimelightHelpers.getTX("limelight-swerve"));
      double velocityY = m_yController.calculate(LimelightHelpers.getTY("limelight-swerve"));
      m_drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(velocityX).withVelocityX(velocityY));

      SmartDashboard.putNumber("X Speed", velocityX);
      SmartDashboard.putNumber("Y Speed", velocityY);
      SmartDashboard.putNumber("Target ID", tagID);

      if (!m_xController.atSetpoint() || !m_yController.atSetpoint() || !m_rotController.atSetpoint()) {
        m_alignedDebounceTimer.reset();
      }
    } else {
      m_drivetrain.setControl(new RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(new RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_invalidTagTimer.hasElapsed(1) || m_alignedDebounceTimer.hasElapsed(0.3);
  }
}
