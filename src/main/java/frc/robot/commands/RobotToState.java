package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pooper.Pooper;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class RobotToState extends SequentialCommandGroup {

    public RobotToState(Elevator elevator, Pooper pooper, Intake intake, RobotStates state) {
        addCommands(
            new ConditionalCommand(
                new ElevatorToHeight(elevator, state).alongWith(new PooperToAngle(pooper, state)),

                new InstantCommand(() -> intake.runIntakePivot(INTAKE_CLEARANCE_POS))
                .until(intake.pivotAtTarget())
                .andThen(new ElevatorToHeight(elevator, state)
                .until(elevator.isMovementSafe())
                .andThen(new InstantCommand(() -> intake.runIntakePivot(INTAKE_RETRACT_POS)))),

                elevator.isMovementSafe()
            )
        );
    }
}
