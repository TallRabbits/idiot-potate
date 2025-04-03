// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.pooper.Pooper;
import static frc.robot.subsystems.pooper.PooperConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PooperToAngle extends Command {
  private final Pooper m_pooper;
  private final RobotStates m_angle;
  /** Creates a new PooperToAngle. */
  public PooperToAngle(Pooper pooper, RobotStates angle) {
    m_pooper = pooper;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_angle) {
      case L1:
      case L2:
      case L3:
        m_pooper.runPooperPivot(PIVOT_SCORING_POS_L1_3);
        break;
      case L4:
        m_pooper.runPooperPivot(PIVOT_SCORING_POS_L4);
        break;
      case DEALGAE_UPPER:
      case DEALGAE_LOWER:
        m_pooper.runPooperPivot(PIVOT_DEALGAE_POS);
        break;
      case BARGE:
        m_pooper.runPooperPivot(PIVOT_SCORING_POS_BARGE);
        break;
      case CORAL_STATION:
        m_pooper.runPooperPivot(PIVOT_CORAL_STATION_POS);
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pooper.pivotAtTarget().getAsBoolean();
  }
}
