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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants.TrenchManeuvering.TrenchLocationWRTBlueOrigin;
import frc.robot.commands.ResetIMU;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbLevel;
import frc.robot.subsystems.ClimberSubsystem.ClimbStates;
import frc.robot.util.BezierCurve;


public class RobotContainer {
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static VisionSubsystem vision = VisionSubsystem.getInstance();
  public final static CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
  public final static ClimberSubsystem climbSub = ClimberSubsystem.getInstance();
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

  public static double maxSpeed = DEFAULT_MAX_SPEED * .5;
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

    configureBindings();

    registeredCommands();
    
    autoChooser = new LoggedDashboardChooser<>("Rebuilt/Auto Chooser");
    configureAutos();
  }

  private void configureAutos() {
    autoChooser.addOption("Outpost + Neutral Ferry", drivetrain.getAutoPath("Outpost + Neutral Ferry", false));
    autoChooser.addOption("Shoot & Ferry", drivetrain.getAutoPath("Shoot & Ferry", false));
    autoChooser.addOption("Ferry Only", drivetrain.getAutoPath("Ferry Only", false));
    autoChooser.addOption("Climb", drivetrain.getAutoPath("Ferry + Climb", false));
    autoChooser.addOption("Odom Test", drivetrain.getAutoPath("Drive Straight", false));
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
      
      // DRIVER.rightTrigger().onTrue(
      //   CoordinationSubsystem.getInstance().setForceTrenchCommand()
      // );


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

    // OPERATOR.povDown().onTrue(new InstantCommand(() -> climbSub.setClimbState(ClimbMasterStates.OVERRIDE)));
    // OPERATOR.povUp().onTrue(new InstantCommand(() -> climbSub.setElevatorOverrideSetpoint(-0.5)).alongWith(new InstantCommand(() -> climbSub.setElevatorRequestConstants(95, 1200))));
    // OPERATOR.povDown().onTrue(new InstantCommand(() -> climbSub.setElevatorOverrideSetpoint(-128)).alongWith(new InstantCommand(() -> climbSub.setElevatorRequestConstants(15, 200))));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> climbSub.setTelescopeOverrideSetpoint(40)).alongWith(new InstantCommand(() -> climbSub.setTelescopingRequestConstants(95, 1400))));
    // OPERATOR.povLeft().onTrue(new InstantCommand(() -> climbSub.setTelescopeOverrideSetpoint(0.1)).alongWith(new InstantCommand(() -> climbSub.setTelescopingRequestConstants(10, 700))));

    // // OPERATOR.x().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().zeroMotors()));
    // OPERATOR.y().onTrue(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB));
    // OPERATOR.a().onTrue(
    //     new InstantCommand(() -> climbSub.setClimbState(ClimbStates.INIT))
    //     .alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.L1)))
    //   );

    // OPERATOR.b().onTrue(climbSub.GoToNextLevel());
    // OPERATOR.x().onTrue(
    //   new InstantCommand(() -> climbSub.setClimbState(ClimbStates.INIT))
    //   .alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)))
    //   );

    // OPERATOR.povLeft().onTrue(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.INIT)).alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.L2))));
    // OPERATOR.povUp().onTrue(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.HANDOFF)).alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.L2))));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.END)).alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.L2))));


    // OPERATOR.a().onTrue(new InstantCommand(() -> climbSub.setElevatorOverrideSetpoint(-128)).alongWith(new InstantCommand(() -> climbSub.setElevatorOverrideSetpoint(0.1))));


    DRIVER.leftTrigger().onTrue(
      CoordinationSubsystem.getInstance().setIntakeCommand()
    );

    // OPERATOR.b().onTrue(CoordinationSubsystem.getInstance().setOverrideCommand(true));
    // OPERATOR.b().onFalse(CoordinationSubsystem.getInstance().setOverrideCommand(false));

    DRIVER.rightBumper().onTrue(
        CoordinationSubsystem.getInstance().setOverrideCommand(true)
      ).onFalse(
        CoordinationSubsystem.getInstance().setOverrideCommand(false)
      );

    OPERATOR.rightTrigger().onTrue(
      CoordinationSubsystem.getInstance().setFireCommand()
    );

    OPERATOR.rightBumper().onTrue(
      CoordinationSubsystem.getInstance().setFastTransferCommand(false)
    ).onFalse(
      CoordinationSubsystem.getInstance().setFastTransferCommand(true)
    );

    OPERATOR.leftTrigger().onTrue(
      CoordinationSubsystem.getInstance().setIntakeCommand()
    );

  
    OPERATOR.b().onTrue(CoordinationSubsystem.getInstance().setForceTrenchCommand());
    OPERATOR.b().onFalse(CoordinationSubsystem.getInstance().setForceTrenchCommand());

    OPERATOR.leftStick().onTrue(
      CoordinationSubsystem.getInstance().setRejectingCommand(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setRejectingCommand(false)
    );

    DRIVER.leftStick().onTrue(
      Commands.either(CoordinationSubsystem.getInstance().setRejectingCommand(true), new InstantCommand(), (() -> DriverStation.isTest()))
      ).onFalse(
        Commands.either(CoordinationSubsystem.getInstance().setRejectingCommand(false), new InstantCommand(), (() -> DriverStation.isTest()))
      );

      DRIVER.rightStick().onTrue(
      Commands.either(CoordinationSubsystem.getInstance().setFireCommand(), new InstantCommand(), (() -> DriverStation.isTest()))
      );
    
    DRIVER.start().onTrue(new ResetIMU(drivetrain));

    DRIVER.back().onTrue(
      new InstantCommand(() -> {IntakeSubsystem.getInstance().zeroEncoders(); ShooterSubsystem.getInstance().zeroEncoder();}).ignoringDisable(true)
    );

  }

  public void registeredCommands() {
    // Intake Commands
    NamedCommands.registerCommand("SetIntake", CoordinationSubsystem.getInstance().setIntakeCommand());
    NamedCommands.registerCommand("SetFire", CoordinationSubsystem.getInstance().setFireCommand());
    NamedCommands.registerCommand("SetHardTrenchOverrideTrue", CoordinationSubsystem.getInstance().setHardTrenchOverride(true));
    NamedCommands.registerCommand("SetHardTrenchOverrideFalse", CoordinationSubsystem.getInstance().setHardTrenchOverride(false));

    NamedCommands.registerCommand("ForceTrench", CoordinationSubsystem.getInstance().setForceTrenchCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
