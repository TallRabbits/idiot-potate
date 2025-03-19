package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pooper.Pooper;

import static frc.robot.Constants.*;

public class RobotToState extends SequentialCommandGroup {

    public RobotToState(Elevator elevator, Pooper pooper, Intake intake, RobotStates state) {
        addCommands(
            new ConditionalCommand(
                new ElevatorToHeight(elevator, state).alongWith(new PooperToAngle(pooper, state)),
                new IntakeClearElevator(elevator, intake).andThen(new ElevatorToHeight(elevator, state)),
                elevator.isMovementSafe()
            )
        );
    }
}
