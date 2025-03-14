package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeConstants {
    public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    static{
        intakeConfig.Slot0.kG = 0.0;
        intakeConfig.Slot0.kS = 0.0;
        intakeConfig.Slot0.kV = 0.0;
        intakeConfig.Slot0.kA = 0.0;
        intakeConfig.Slot0.kP = 0.0;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;
    }
    
}
