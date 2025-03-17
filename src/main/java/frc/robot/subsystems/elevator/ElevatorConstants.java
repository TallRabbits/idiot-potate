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
        elevatorConfig.Slot0.kG = 0.0;
        elevatorConfig.Slot0.kS = 0.0;
        elevatorConfig.Slot0.kV = 0.0;
        elevatorConfig.Slot0.kA = 0.0;
        elevatorConfig.Slot0.kP = 0.0;
        elevatorConfig.Slot0.kI = 0.0;
        elevatorConfig.Slot0.kD = 0.0;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = 2.5;
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.Feedback.SensorToMechanismRatio = 133.33;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.19;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    }
}
