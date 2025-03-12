package frc.robot.subsystems.pooper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class PooperConstants {
    public static final TalonFXConfiguration coralConfig = new TalonFXConfiguration();
    static {
        coralConfig.Slot0.kG = 0.0;
        coralConfig.Slot0.kS = 0.0;
        coralConfig.Slot0.kV = 0.0;
        coralConfig.Slot0.kA = 0.0;
        coralConfig.Slot0.kP = 0.0;
        coralConfig.Slot0.kI = 0.0;
        coralConfig.Slot0.kP = 0.0;
    }

public static final TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
    static {
        algaeConfig.Slot0.kG = 0.0;
        algaeConfig.Slot0.kS = 0.0;
        algaeConfig.Slot0.kV = 0.0;
        algaeConfig.Slot0.kA = 0.0;
        algaeConfig.Slot0.kP = 0.0;
        algaeConfig.Slot0.kI = 0.0;
        algaeConfig.Slot0.kP = 0.0;
    }

    public static final TalonFXConfiguration pooperPivotConfig = new TalonFXConfiguration();
    static {
        pooperPivotConfig.Slot0.kG = 0.0;
        pooperPivotConfig.Slot0.kS = 0.0;
        pooperPivotConfig.Slot0.kV = 0.0;
        pooperPivotConfig.Slot0.kA = 0.0;
        pooperPivotConfig.Slot0.kP = 0.0;
        pooperPivotConfig.Slot0.kI = 0.0;
        pooperPivotConfig.Slot0.kP = 0.0;
    }
}
