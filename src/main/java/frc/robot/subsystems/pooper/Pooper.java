package frc.robot.subsystems.pooper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.pooper.PooperConstants.*;

public class Pooper extends SubsystemBase {
    private final TalonFX coralRoller = new TalonFX(0);
    private final TalonFX algaeRoller = new TalonFX(0);
    private final TalonFX pooperPivot = new TalonFX(0);

    private final CANrange coralSensor = new CANrange(0);

    private final VelocityVoltage coralRollerRequest = new VelocityVoltage(null);
    private final VelocityVoltage algaeRollerRequest = new VelocityVoltage(null);
    private final MotionMagicVoltage pooperPivotRequest = new MotionMagicVoltage(null);
    private final NeutralOut neutral = new NeutralOut();

    private final StatusSignal<Boolean> coralSensorHasCoral = coralSensor.getIsDetected(true);
    private final StatusSignal<Current> algaeMotorStall = algaeRoller.getMotorStallCurrent();
    private final StatusSignal<Current> algaeMotorStatorCurrent = algaeRoller.getStatorCurrent();
    private final StatusSignal<AngularVelocity> algaeRollerVelocity = algaeRoller.getVelocity();

    public Pooper() {
        coralRoller.getConfigurator().apply(PooperConstants.coralConfig);
        algaeRoller.getConfigurator().apply(PooperConstants.algaeConfig);
        pooperPivot.getConfigurator().apply(PooperConstants.pooperPivotConfig);
    }

    public void setCoralRoller(double rps) {
        coralRoller.setControl(coralRollerRequest.withVelocity(rps));
    }

    public void setAlgaeRoller(double rps) {
        algaeRoller.setControl(algaeRollerRequest.withVelocity(rps));
    }

    public void setPooperPivot(double angle) {
        pooperPivot.setControl(pooperPivotRequest.withPosition(angle));
    }

    public void stopCoral() {
        coralRoller.setControl(coralRollerRequest);
    }

    public void stopAlgae() {
        algaeRoller.setControl(algaeRollerRequest);
    }

    public boolean hasCoral() {
        if (coralSensorHasCoral.getValue()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isAlgaeStalled() {
        if ((algaeMotorStall.getValueAsDouble() < algaeMotorStatorCurrent.getValueAsDouble()) && (algaeRollerVelocity.getValueAsDouble() < 5)) {
            return true;
        } else {
            return false;
        }
    }

    
                

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pooper/Coral Sensor/Has Coral", coralSensorHasCoral.getValue());
    }
}

    

    




