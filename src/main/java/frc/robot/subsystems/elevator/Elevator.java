package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;hfeihgheedhdvvttgrnvttcvtgifculukkujgnltkuhu


public class Elevator extends SubsystemBase {
    // Elevator motors
    private final TalonFX leaderElevator;
    private final TalonFX followerElevator;
    // Elevator control requests
    private final MotionMagicVoltage leaderElevatorRequest;
    private final Follower followerElevatorRequest;
    // Motor Telemetry
    private final StatusSignal<Double> leaderElevatorPosition;
    private final StatusSignal<Double> leaderElevatorSupplyCurrnet;
    private final StatusSignal<Double> leaderElevatorTempC;

    private final StatusSignal<Double> followerElevatorPosition;
    private final StatusSignal<Double> followerElevatorSupplyCurrent;
    private final StatusSignal<Double> followerElevatorTempC;


    public Elevator() {
        // Elevator motors
        leaderElevator = new TalonFX(LEADER_ELEVATOR_ID, CANBUS_NAME);
        followerElevator = new TalonFX(FOLLOWER_ELEVATOR_ID, CANBUS_NAME);

        leaderElevatorRequest = new MotionMagicVoltage(0.0);
        followerElevatorRequest = new Follower(LEADER_ELEVATOR_ID);


    }




}
