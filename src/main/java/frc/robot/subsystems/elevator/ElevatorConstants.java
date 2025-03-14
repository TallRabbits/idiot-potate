package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ElevatorConstants {
    public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    static {
        elevatorConfig.Slot0.kG = 0.0;
        elevatorConfig.Slot0.kS = 0.0;
        elevatorConfig.Slot0.kV = 0.0;
        elevatorConfig.Slot0.kA = 0.0;
        elevatorConfig.Slot0.kP = 0.0;
        elevatorConfig.Slot0.kI = 0.0;
        elevatorConfig.Slot0.kD = 0.0;
    }
    
}
