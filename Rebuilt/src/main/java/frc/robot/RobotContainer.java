// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants.TrenchManeuvering.TrenchLocationWRTBlueOrigin;
import frc.robot.commands.GoToPosCommand;
import frc.robot.commands.ResetIMU;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.TransferSubsystem.TransferState;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import frc.robot.util.BezierCurve;
import frc.robot.util.FuelSim;


public class RobotContainer {
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static VisionSubsystem vision = VisionSubsystem.getInstance();
  public final static CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();

  public static TurretSubsystem turretSub = TurretSubsystem.getInstance();
  public static ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
  public static TransferSubsystem transferSub = TransferSubsystem.getInstance();
  public static IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();
  public static LEDSubsystem ledSub = LEDSubsystem.getInstance();

  public static FuelSim fuelSim = new FuelSim(); // creates a new fuelSim of FuelSim
  
  public BezierCurve driveBezier = new BezierCurve("drive", 117.4, 0.054, 91.4, 0.76, 0.1, 0.01);
  public BezierCurve rotateBezier = new BezierCurve("rotate", 120.1, 0.145, 92.2, 0.362, 0.035, 0.03);

  private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(1.125).in(RadiansPerSecond);

  public static double maxSpeed = DEFAULT_MAX_SPEED * 0.4;
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

    // fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone

    Supplier<ChassisSpeeds> fieldCentricSupplier = () -> drivetrain.getState().Speeds;
    Supplier<Pose2d> poseSupplier = () -> drivetrain.getState().Pose;
      // Register a robot for collision with fuel
      fuelSim.registerRobot(
              0.6858, // from left to right in meters
              0.6858, // from front to back in meters
              .01, // from floor to top of bumpers in meters
              poseSupplier, // Supplier<Pose2d> of robot pose
              fieldCentricSupplier); // Supplier<ChassisSpeeds> of field-centric chassis speeds

    fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)
    configureBindings();
    registeredCommands();

    configureAutos();
  }

  private void configureAutos() {
    autoChooser.addOption("Outpost + Neutral Ferry", drivetrain.getAutoPath("Outpost + Neutral Ferry", false));
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
    // DRIVER.povUp().onTrue(new InstantCommand(() -> shooterSub.setOverrideHoodAngle(3)).andThen(new InstantCommand(() ->shooterSub.setS(true))));
    // // DRIVER.povUp().onFalse(new InstantCommand(() -> shooterSub.setOverrideMode(false)));
    
    DRIVER.povDown().onTrue(new InstantCommand(() -> shooterSub.setOverrideHoodAngle(0)));
    // DRIVER.povDown().onFalse(new InstantCommand(() -> shooterSub.setOverrideMode(false)));

    // For tuning hood angle PID
    DRIVER.povUp().onTrue(new InstantCommand(() -> shooterSub.setState(ShooterState.NOT_SHOOTING, AngleState.AIMING)).andThen(
      new InstantCommand(() -> shooterSub.setOverrideHoodAngle(40)))
    .andThen(new InstantCommand(() ->shooterSub.setOverrideModeAngle(true))));

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
    NamedCommands.registerCommand("SetIntake", coordSub.setIntakeCommand());
    NamedCommands.registerCommand("SetFire", coordSub.setFireCommand());

    }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
