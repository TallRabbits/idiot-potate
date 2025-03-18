package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
    private final VoltageOut sysIdControl = new VoltageOut(0.0);

    // Motor Telemetry
    private final StatusSignal<Angle> elevatorLeaderPosition;
    private final StatusSignal<AngularVelocity> elevatorLeaderVelocity;
    private final StatusSignal<Current> elevatorLeaderSupplyCurrent;
    private final StatusSignal<Temperature> elevatorLeaderTempC;

    private final StatusSignal<Angle> elevatorFollowerPosition;
    private final StatusSignal<AngularVelocity> elevatorFollowerVelocity;
    private final StatusSignal<Current> elevatorFollowerSupplyCurrent;
    private final StatusSignal<Temperature> elevatorFollowerTempC;

    private final SysIdRoutine sysIdRoutine = 
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                    // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> elevatorLeader.setControl(sysIdControl.withOutput(volts)),
            null,
            this
        )
    );

    public Elevator() {
        elevatorLeader.getConfigurator().apply(currentLimit);
        elevatorLeader.getConfigurator().apply(elevatorConfig);
        elevatorLeader.setPosition(0);
        elevatorLeaderPosition = elevatorLeader.getPosition();
        elevatorLeaderVelocity = elevatorLeader.getVelocity();
        elevatorLeaderSupplyCurrent = elevatorLeader.getSupplyCurrent();
        elevatorLeaderTempC = elevatorLeader.getDeviceTemp();

        elevatorFollower.setControl(elevatorFollowerRequest);
        elevatorFollowerPosition = elevatorFollower.getPosition();
        elevatorFollowerVelocity = elevatorFollower.getVelocity();
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
    
    public void elevatorToPosition(double height){
        elevatorLeader.setControl(elevatorLeaderRequest.withPosition(height));
    }

    public void detectStallAndReset(){
        if (elevatorLeaderSupplyCurrent.getValueAsDouble() > HIGH_CURRENT_THRESHOLD){
            if (elevatorLeaderVelocity.getValueAsDouble() < LOW_VELOCITY_THRESHOLD){
                elevatorLeader.setPosition(0);
            }
        }
    }

    public void stop() {
        elevatorLeader.setControl(neutral);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction ) { 
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorLeaderPosition, 
            elevatorLeaderVelocity, 
            elevatorLeaderSupplyCurrent,
            elevatorLeaderTempC, 
            elevatorFollowerPosition, 
            elevatorFollowerVelocity, 
            elevatorFollowerSupplyCurrent, 
            elevatorFollowerTempC
        );

        SmartDashboard.putNumber("Elevator Leader Position", elevatorLeaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Velocity", elevatorLeaderVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Supply Current", elevatorLeaderSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Temp C", elevatorLeaderTempC.getValueAsDouble());

        SmartDashboard.putNumber("Elevator Follower Position", elevatorFollowerPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Velocity", elevatorFollowerVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Supply Current", elevatorFollowerSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Temp C", elevatorFollowerTempC.getValueAsDouble());
    }

}
