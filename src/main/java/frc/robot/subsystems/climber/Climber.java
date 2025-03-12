package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX climberLeader = new TalonFX(0);
    private final TalonFX climberFollower = new TalonFX(0);

    
}
