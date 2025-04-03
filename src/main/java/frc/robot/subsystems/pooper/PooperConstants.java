package frc.robot.subsystems.pooper;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        pooperPivotConfig.Slot0.kS = 0;
        pooperPivotConfig.Slot0.kV = 0;
        pooperPivotConfig.Slot0.kA = 0.0;
        pooperPivotConfig.Slot0.kP = 1.0;
        pooperPivotConfig.Slot0.kI = 0.0;
        pooperPivotConfig.Slot0.kD = 0.0;
        pooperPivotConfig.MotionMagic.MotionMagicAcceleration = 85;
        pooperPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 20;
        pooperPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pooperPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        pooperPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        pooperPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        pooperPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    }

    public static final CANrangeConfiguration coralSensorConfig = new CANrangeConfiguration();
    static {
        coralSensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 4000;
        coralSensorConfig.ProximityParams.ProximityHysteresis = 0.01;
        coralSensorConfig.ProximityParams.ProximityThreshold = 0.15;
    }

    public static final double CORAL_SCORING_VOLTS = 0.0;
    public static final double CORAL_INTAKE_VOLTS = 0.0;

    public static final double PIVOT_INTAKE_TRANSFER_POS = -1.0;

    public static final double PIVOT_SCORING_POS_L1_3 = 4.5;
    public static final double PIVOT_SCORING_POS_L4 = 1.3;

    public static final double PIVOT_SCORING_POS_BARGE = 0.0;

    public static final double PIVOT_DEALGAE_POS = 0.0;

    public static final double PIVOT_CORAL_STATION_POS = 0.5;
}
