package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
    private final StatusSignal<Angle> elevatorLeaderPosition;
    private final StatusSignal<AngularVelocity> elevatorLeaderVelocity;
    private final StatusSignal<Current> elevatorLeaderSupplyCurrent;

    private final StatusSignal<Angle> elevatorFollowerPosition;
    private final StatusSignal<AngularVelocity> elevatorFollowerVelocity;

    public Elevator() {
        elevatorLeader.getConfigurator().apply(currentLimit);
        elevatorLeader.getConfigurator().apply(elevatorConfig);
        elevatorLeader.setPosition(0);
        elevatorLeader.setControl(neutral);
        elevatorLeaderPosition = elevatorLeader.getPosition();
        elevatorLeaderVelocity = elevatorLeader.getVelocity();
        elevatorLeaderSupplyCurrent = elevatorLeader.getSupplyCurrent();

        elevatorFollower.setControl(elevatorFollowerRequest);
        elevatorFollowerPosition = elevatorFollower.getPosition();
        elevatorFollowerVelocity = elevatorFollower.getVelocity();
    }

    public void elevatorNudgeUp() {
        var curAng = elevatorLeader.getPosition().getValueAsDouble();
        elevatorLeader.setControl(elevatorLeaderRequest.withPosition(curAng + 0.01));
    }

    public void elevatorNudgeDown() {
        var curAng = elevatorLeader.getPosition().getValueAsDouble();
        elevatorLeader.setControl(elevatorLeaderRequest.withPosition(curAng - 0.01));
    }

    public void elevatorToPosition(double height) {
        elevatorLeader.setControl(elevatorLeaderRequest.withPosition(height));
    }

    public boolean elevatorAtTarget(RobotStates m_height) {
        switch (m_height) {
            case L1:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_L1_POS) < 0.1;
            case L2:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_L2_POS) < 0.1;        
            case L3:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_L3_POS) < 0.1;        
            case L4:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_L4_POS) < 0.1;        
            case DEALGAE_UPPER:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_DEALGAE_UPPER_POS) < 0.1;        
            case DEALGAE_LOWER:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_DEALGAE_LOWER_POS) < 0.1;        
            case BARGE:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_BARGE_POS) < 0.1;        
            case CORAL_STATION:
              return Math.abs(this.getElevatorHeight() - ELEVATOR_CORAL_STATION_POS) < 0.1;        
            default:
              return false;
        }
    }

    public double getElevatorHeight() {
        return elevatorLeaderPosition.getValueAsDouble();
    }

    public void detectStallAndReset() {
        if (elevatorLeaderSupplyCurrent.getValueAsDouble() > HIGH_CURRENT_THRESHOLD) {
            if (elevatorLeaderVelocity.getValueAsDouble() < LOW_VELOCITY_THRESHOLD) {
                elevatorLeader.setPosition(0);
            }
        }
    }

    public void resetElevator() {
      elevatorLeader.setPosition(0);
    }

    public void stop() {
        elevatorLeader.setControl(neutral);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorLeaderPosition, 
            elevatorLeaderVelocity, 
            elevatorFollowerPosition, 
            elevatorFollowerVelocity
        );

        SmartDashboard.putNumber("Elevator Leader Position", elevatorLeaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Velocity", elevatorLeaderVelocity.getValueAsDouble());

        SmartDashboard.putNumber("Elevator Follower Position", elevatorFollowerPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Velocity", elevatorFollowerVelocity.getValueAsDouble());
    }

}
