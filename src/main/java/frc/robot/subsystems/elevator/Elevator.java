package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLeader = new TalonFX(0);
    private final TalonFX elevatorFollower = new TalonFX(0); 

    private final MotionMagicVoltage elevatorLeaderRequest = new MotionMagicVoltage(null);
    private final Follower elevatorFollowerRequest = new Follower(0, false);
}
