package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeConstants {
    public static final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    static{
        rollerConfig.Slot0.kG = 0.0;
        rollerConfig.Slot0.kS = 0.0;
        rollerConfig.Slot0.kV = 0.0;
        rollerConfig.Slot0.kA = 0.0;
        rollerConfig.Slot0.kP = 0.0;
        rollerConfig.Slot0.kI = 0.0;
        rollerConfig.Slot0.kD = 0.0;
    }

    public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration(); 
    static {
        pivotConfig.Slot0.kG = 0.0;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kV = 0.0;
        pivotConfig.Slot0.kA = 0.0;
        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
    }
    
}
