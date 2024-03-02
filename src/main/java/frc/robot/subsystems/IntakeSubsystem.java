package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{
    /* TalonFX intake_motor = new TalonFX(Constants.IntakeSubsystemConstants.intake_motor);
    TalonFX roller_motor1 = new TalonFX(Constants.IntakeSubsystemConstants.roller_motor1);
    TalonFX roller_motor2 = new TalonFX(Constants.IntakeSubsystemConstants.roller_motor2); */
    CANSparkMax intake_motor = new CANSparkMax(Constants.IntakeSubsystemConstants.intake_motor, MotorType.kBrushed);
    

    public IntakeSubsystem()
    {
        intake_motor.setInverted(false);
        /* roller_motor1.setInverted(false);
        roller_motor2.setInverted(false); */
    }

    public void setPower(double power)
    {
        intake_motor.set(power);
        /* roller_motor1.set(power);
        roller_motor2.set(power); */
    }

    public void off()
    {
        intake_motor.set(0);
        /* roller_motor1.set(0);
        roller_motor2.set(0); */
    }
}
