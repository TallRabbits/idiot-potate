package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pooper.Pooper;

import static frc.robot.Constants.*;

public class RobotToState extends SequentialCommandGroup {

    public RobotToState(Elevator elevator, Pooper pooper, RobotStates state) {
        addCommands(
            new ElevatorToHeight(elevator, state).alongWith(new PooperToAngle(pooper, state))
        );
    }
}
