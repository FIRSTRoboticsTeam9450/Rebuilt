// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.BezierCurve;


public class RobotContainer {
  public static TurretSubsystem turretSub = TurretSubsystem.getInstance();
  public static ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();

  public BezierCurve driveBezier = new BezierCurve("drive", 117.4, 0.054, 91.4, 0.76, 0.1, 0.01);
  public BezierCurve rotateBezier = new BezierCurve("rotate", 120.1, 0.145, 92.2, 0.362, 0.035, 0.03);

  private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(1.125).in(RadiansPerSecond);

  public static double maxSpeed = DEFAULT_MAX_SPEED;
  public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE;

  private final CommandXboxController DRIVER = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController OPERATOR = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(driveBezier.getDeadband())
        .withRotationalDeadband(rotateBezier.getDeadband())    
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public LoggedDashboardChooser<Command> autoChooser;

  public static double pigeonOffset = 0;

  public RobotContainer() {

    turretSub.setDriverController(DRIVER);

    autoChooser = new LoggedDashboardChooser<>("Rebuilt/Auto Chooser");

    configureBindings();
  }

  private void configureAutos() {
    
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    
    // For tuning hood angle PID
    DRIVER.povUp().onTrue(new InstantCommand(() -> shooterSub.setOverrideHoodAngle(3)).andThen(new InstantCommand(() ->shooterSub.setOverrideMode(true))));
    DRIVER.povUp().onFalse(new InstantCommand(() -> shooterSub.setOverrideMode(false)));
    
    DRIVER.povDown().onTrue(new InstantCommand(() -> shooterSub.setOverrideHoodAngle(0)).andThen(new InstantCommand(() ->shooterSub.setOverrideMode(true))));
    DRIVER.povDown().onFalse(new InstantCommand(() -> shooterSub.setOverrideMode(false)));
    
    
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
