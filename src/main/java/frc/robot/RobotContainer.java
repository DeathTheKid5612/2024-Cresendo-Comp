// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final ShooterSubsystem shooter_sub = new ShooterSubsystem();
  private final IntakeSubsystem intake_sub = new IntakeSubsystem();
  //private final ClimberSubsystem climber_sub = new ClimberSubsystem();

  private final Command c_shoot = new InstantCommand( () -> shooter_sub.setPower(Constants.ShooterSubsystemConstants.shooter_power) );
  private final Command c_stop_shoot = new InstantCommand( () -> shooter_sub.shooterOff() );

  private final Command c_intake_in = new InstantCommand( () -> intake_sub.setPower(Constants.IntakeSubsystemConstants.intake_power) );
  private final Command c_intake_out = new InstantCommand( () -> intake_sub.setPower(-Constants.IntakeSubsystemConstants.intake_power) );
  private final Command c_intake_off = new InstantCommand( () -> intake_sub.off() );

 /*  private final Command c_climber_up = new InstantCommand( () -> climber_sub.setPower(Constants.IntakeSubsystemConstants.intake_power) );
  private final Command c_climber_down = new InstantCommand( () -> climber_sub.setPower(-Constants.IntakeSubsystemConstants.intake_power) );
  private final Command c_climber_off = new InstantCommand( () -> climber_sub.off() ); */


  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity

  

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //shooter_sub.setDefaultCommand(c_stop_shoot);
    joystick.rightBumper().onTrue(c_shoot).onFalse(c_stop_shoot);

    joystick.a().onTrue(c_intake_in).onFalse(c_intake_off);
    joystick.leftBumper().onTrue(c_intake_out).onFalse(c_intake_off);

    /* joystick.povUp().onTrue(c_climber_up).onFalse(c_climber_off);
    joystick.povDown().onTrue(c_climber_down).onFalse(c_climber_off); */

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  public void resetPose() {
    drivetrain.seedFieldRelative();
  }

  /* public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto 
    //return autoChooser.getSelected();
    return Command
  } */
}
