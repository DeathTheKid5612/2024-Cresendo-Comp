package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    TalonFX climb_motor1 = new TalonFX(Constants.ClimberSubsystemConstants.climb_motor1);
    TalonFX climb_motor2 = new TalonFX(Constants.ClimberSubsystemConstants.climb_motor2);

    public ClimberSubsystem()
    {
        climb_motor1.setInverted(false);
        climb_motor2.setInverted(false);
    }

    public void setPower(double power)
    {
        climb_motor1.set(power);
        climb_motor2.set(power);
    }

    public void off()
    {
        climb_motor1.set(0);
        climb_motor2.set(0);    
    }
}
