package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.*;

public class ElevatorConstants {
    public static final double HIGH_CURRENT_THRESHOLD = 120;
    public static final double LOW_VELOCITY_THRESHOLD = 1.0;

    public static final TalonFXConfiguration currentLimit = new TalonFXConfiguration();
    static {
        currentLimit.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        currentLimit.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    static {
        elevatorConfig.Slot0.kG = 0.55;
        elevatorConfig.Slot0.kS = 0.325;
        elevatorConfig.Slot0.kV = 0.72;
        elevatorConfig.Slot0.kA = 0.0;
        elevatorConfig.Slot0.kP = 185;
        elevatorConfig.Slot0.kI = 0.0;
        elevatorConfig.Slot0.kD = 0.0;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = 4.5;
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.Feedback.SensorToMechanismRatio = 6;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.0;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    }

    public static final double ELEVATOR_INTAKE_TRANSFER_POS = 0.0;

    public static final double ELEVATOR_INTAKE_LOWER_DANGER_POS = 0.0;
    public static final double ELEVATOR_INTAKE_UPPER_DANGER_POS = 0.0;

    public static final double ELEVATOR_L1_POS = 0.65;
    public static final double ELEVATOR_L2_POS = 1.05;
    public static final double ELEVATOR_L3_POS = 2.03;
    ;
    public static final double ELEVATOR_L4_POS = 3.9;

    public static final double ELEVATOR_DEALGAE_UPPER_POS = 0.0;
    public static final double ELEVATOR_DEALGAE_LOWER_POS = 0.0;

    public static final double ELEVATOR_BARGE_POS = 0.0;

    public static final double ELEVATOR_CORAL_STATION_POS = 0.0;



    
}
