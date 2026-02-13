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
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants.TrenchManeuvering.TrenchLocationWRTBlueOrigin;
import frc.robot.commands.ResetIMU;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BezierCurve;


public class RobotContainer {
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static VisionSubsystem vision = VisionSubsystem.getInstance();
  public final static CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();

  public static TurretSubsystem turretSub = TurretSubsystem.getInstance();
  public static ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
  public static TransferSubsystem transferSub = TransferSubsystem.getInstance();
  public static IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();
  public static LEDSubsystem ledSub = LEDSubsystem.getInstance();


  /**
   * Woojins Drive: 117.4, 0.054, 91.4, 0.76, 0.1, 0.01
   * Woojins Rotate: 120.1, 0.145, 92.2, 0.362, 0.035, 0.03
   */
  public BezierCurve driveBezier = new BezierCurve("drive", 88.2, 0.035, 92, 0.839, 0.1, 0.01);
  public BezierCurve rotateBezier = new BezierCurve("rotate", 97.3, 0.193, 72.1, 0.822, 0.035, 0.03);

  private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(1.125).in(RadiansPerSecond);

  public static double maxSpeed = DEFAULT_MAX_SPEED * 0.5; //0.4 for 40%
  public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE * 1;

  private final CommandXboxController DRIVER = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController OPERATOR = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(driveBezier.getDeadband())
        .withRotationalDeadband(rotateBezier.getDeadband())    
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public LoggedDashboardChooser<Command> autoChooser;

  public static double pigeonOffset = 0;

  public RobotContainer() {

    TurretSubsystem.getInstance().setDriverController(DRIVER);

    autoChooser = new LoggedDashboardChooser<>("Rebuilt/Auto Chooser");

    configureBindings();
    registeredCommands();

    configureAutos();
  }

  private void configureAutos() {
    autoChooser.addOption("Outpost + Neutral Ferry", drivetrain.getAutoPath("Outpost + Neutral Ferry", false));
    autoChooser.addOption("Shoot & Ferry", drivetrain.getAutoPath("Shoot & Ferry", false));
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
    // DRIVER.povDown().onTrue(new InstantCommand(() -> shooterSub.setOverrideHoodAngle(0)).andThen(new InstantCommand(() -> shooterSub.setOverrideModeAngle(true))));// .andThen(new InstantCommand(() ->shooterSub.setS(true))));
    // DRIVER.povUp().onFalse(new InstantCommand(() -> shooterSub.setOverrideMode(false)));
    
    // DRIVER.povDown().onTrue(new InstantCommand(() -> shooterSub.setOverrideHoodAngle(0)));
    // DRIVER.povDown().onFalse(new InstantCommand(() -> shooterSub.setOverrideMode(false)));

    // For tuning hood angle PID
    // DRIVER.povUp().onTrue(new InstantCommand(() -> shooterSub.setState(ShooterState.NOT_SHOOTING, AngleState.AIMING)).andThen(
    //   new InstantCommand(() -> shooterSub.setOverrideHoodAngle(40)))
    // .andThen(new InstantCommand(() ->shooterSub.setOverrideModeAngle(true))));

    // DRIVER.povRight().onTrue(shooterSub.setAngleState(AngleState.ZEROING));
    // DRIVER.povDown().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.CLOSE_LEFT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );
    // DRIVER.povLeft().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.FAR_LEFT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );
    // DRIVER.povRight().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.CLOSE_RIGHT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );
    // DRIVER.povUp().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.FAR_RIGHT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );

    DRIVER.rightTrigger().onTrue(
      CoordinationSubsystem.getInstance().setFireCommand()
    );

    DRIVER.leftTrigger().onTrue(
      CoordinationSubsystem.getInstance().setIntakeCommand()
    );

    DRIVER.b().onTrue(CoordinationSubsystem.getInstance().setOverrideCommand(true));
    DRIVER.b().onFalse(CoordinationSubsystem.getInstance().setOverrideCommand(false));
    
    DRIVER.start().onTrue(new ResetIMU(drivetrain));

    DRIVER.back().onTrue(
      new InstantCommand(() -> {IntakeSubsystem.getInstance().zeroEncoders(); ShooterSubsystem.getInstance().zeroEncoder();}).ignoringDisable(true)
    );

  }

  public void registeredCommands() {
    // Intake Commands
    NamedCommands.registerCommand("SetIntake", CoordinationSubsystem.getInstance().setIntakeCommand());
    NamedCommands.registerCommand("SetFire", CoordinationSubsystem.getInstance().setFireCommand());

    }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
