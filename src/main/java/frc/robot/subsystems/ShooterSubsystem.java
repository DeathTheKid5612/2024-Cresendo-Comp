package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    TalonFX motor1 = new TalonFX(Constants.ShooterSubsystemConstants.motor1_id);
    TalonFX motor2 = new TalonFX(Constants.ShooterSubsystemConstants.motor2_id);
    TalonFX motor3 = new TalonFX(Constants.ShooterSubsystemConstants.motor3_id);
    TalonFX motor4 = new TalonFX(Constants.ShooterSubsystemConstants.motor4_id);

    public ShooterSubsystem()
    {
        motor3.setInverted(true);
        motor4.setInverted(true);
    }

    public void setPower(double power)
    {
        motor1.set(power);
        motor2.set(power);
        motor3.set(power);
        motor4.set(power);
    }

    public void shooterOff()
    {
        motor1.set(0);
        motor2.set(0);
        motor3.set(0);
        motor4.set(0);
    }
}
