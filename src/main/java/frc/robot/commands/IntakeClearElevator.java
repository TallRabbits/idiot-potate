// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import static frc.robot.subsystems.intake.IntakeConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeClearElevator extends Command {
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Debouncer m_debounce = new Debouncer(0.1, DebounceType.kBoth);

  /** Creates a new IntakeClearElevator. */
  public IntakeClearElevator(Elevator elevator, Intake intake) {
    m_elevator = elevator;
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runIntakePivot(INTAKE_CLEARANCE_POS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntakePivot(INTAKE_RETRACT_POS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_debounce.calculate(m_elevator.elevatorAtTarget())||;
  }
}
