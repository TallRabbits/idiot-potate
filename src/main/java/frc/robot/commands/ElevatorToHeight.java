// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.elevator.Elevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToHeight extends Command {
  /** Creates a new ElevatorToHeight. */
  private final Elevator m_elevator;
  private final RobotStates m_height;

  public ElevatorToHeight(Elevator elevator, RobotStates height) {
    m_elevator = elevator;
    m_height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_height) {
      case INTAKE_TRANSFER:
        m_elevator.elevatorToPosition(ELEVATOR_INTAKE_TRANSFER_POS);
        break;
      case L1:
        m_elevator.elevatorToPosition(ELEVATOR_L1_POS);
        break;
      case L2:
        m_elevator.elevatorToPosition(ELEVATOR_L2_POS);
        break;
      case L3:
        m_elevator.elevatorToPosition(ELEVATOR_L3_POS);
        break;
      case L4:
        m_elevator.elevatorToPosition(ELEVATOR_L4_POS);
        break;
      case DEALGAE_UPPER:
        m_elevator.elevatorToPosition(ELEVATOR_DEALGAE_UPPER_POS);
        break;
      case DEALGAE_LOWER:
        m_elevator.elevatorToPosition(ELEVATOR_DEALGAE_LOWER_POS);
        break;
      case BARGE:
        m_elevator.elevatorToPosition(ELEVATOR_BARGE_POS);
        break;
      case CORAL_STATION:
        m_elevator.elevatorToPosition(ELEVATOR_CORAL_STATION_POS);
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
    return m_elevator.elevatorAtTarget();
  }
}