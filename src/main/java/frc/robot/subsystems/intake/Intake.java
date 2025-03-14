package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase{
    private final TalonFX intakeRoller = new TalonFX(0);
    private final TalonFX intakePivot = new TalonFX(0);

    private final VelocityVoltage intakeRollerRequesnt = new VelocityVoltage(null);
    private final MotionMagicVoltage intakePivotRequest = new MotionMagicVoltage(null);
    private final NeutralOut neutral = new NeutralOut();
     
    public Intake(){
        intakeRoller.getConfigurator().apply(IntakeConstants.intakeConfig);
        intakePivot.getConfigurator().apply(IntakeConstants.intakeConfig);
    }
}
