// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pooper.Pooper;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.pooper.PooperConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  private final Elevator m_elevator;
  private final Pooper m_pooper;
  /** Creates a new ScoreCoral. */
  public ScoreCoral(Elevator elevator, Pooper pooper) {
    m_elevator = elevator;
    m_pooper = pooper;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getElevatorHeight() > ELEVATOR_L3_POS + 0.5) {
      m_pooper.runCoralRoller(CORAL_SCORING_VOLTS_L4);
    } else {
      m_pooper.runCoralRoller(CORAL_SCORING_VOLTS_L1_3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pooper.stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_pooper.hasCoral().getAsBoolean();
  }
}
