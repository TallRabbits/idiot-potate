package frc.robot.subsystems.pooper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

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

    public Pooper() {
        coralRoller.getConfigurator().apply(PooperConstants.coralConfig);
        algaeRoller.getConfigurator().apply(PooperConstants.algaeConfig);
        pooperPivot.getConfigurator().apply(PooperConstants.pooperPivotConfig);
    }


}


