package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;


public class Elevator extends SubsystemBase {
    // Elevator motors
    private final TalonFX elevatorLeader = new TalonFX(ELEVATOR_LEADER_ID, CANBUS_RIO);
    private final TalonFX elevatorFollower = new TalonFX(ELEVATOR_FOLLOWER_ID, CANBUS_RIO); 

    // Neutral request, determined by NeutralModeValue of motor
    private final NeutralOut neutral = new NeutralOut();

    // Elevator control requests
    private final MotionMagicVoltage elevatorLeaderRequest = new MotionMagicVoltage(0.0);
    private final Follower elevatorFollowerRequest = new Follower(ELEVATOR_LEADER_ID, false);

    // Motor Telemetry
    private final StatusSignal<Double> elevatorLeaderPosition;
    private final StatusSignal<Double> elevatorLeaderSupplyCurrent;
    private final StatusSignal<Double> elevatorLeaderTempC;

    private final StatusSignal<Double> elevatorFollowerPosition;
    private final StatusSignal<Double> elevatorFollowerSupplyCurrent;
    private final StatusSignal<Double> elevatorFollowerTempC;

    public Elevator() {
        elevatorLeader.getConfigurator().apply(currentLimit);
        elevatorLeader.getConfigurator().apply(elevatorConfig);
        elevatorLeader.setPosition(0);
        elevatorLeaderPosition = elevatorLeader.getPosition();
        elevatorLeaderSupplyCurrent = elevatorLeader.getSupplyCurrent();
        elevatorLeaderTempC = elevatorLeader.getDeviceTemp();

        elevatorFollower.setControl(elevatorFollowerRequest);
        elevatorFollowerPosition = elevatorFollower.getPosition();
        elevatorFollowerSupplyCurrent = elevatorFollower.getSupplyCurrent();
        elevatorFollowerTempC = elevatorFollower.getDeviceTemp();
    }

    public void elevatorUp() {
        var curAng = elevatorLeader.getPosition().getValueAsDouble();
        elevatorLeader.setControl(elevatorLeaderRequest.withPosition(curAng + 0.01));
    }

    public void elevatorDown() {
        var curAng = elevatorLeader.getPosition().getValueAsDouble();
        elevatorLeader.setControl(elevatorLeaderRequest.withPosition(curAng - 0.01));
    }
    
    public void stop() {
        elevatorLeader.setControl(neutral);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorLeaderPosition, 
            elevatorLeaderSupplyCurrent,
            elevatorLeaderTempC, 
            elevatorFollowerPosition, 
            elevatorFollowerSupplyCurrent, 
            elevatorFollowerTempC
        );

        SmartDashboard.putNumber("Elevator Leader Position", elevatorLeaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Supply Current", elevatorLeaderSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Temp C", elevatorLeaderTempC.getValueAsDouble());

        SmartDashboard.putNumber("Elevator Follower Position", elevatorFollowerPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Supply Current", elevatorFollowerSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Temp C", elevatorFollowerTempC.getValueAsDouble());
    }

}
