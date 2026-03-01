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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlignPos;
import frc.robot.commands.PositionAlignCommand;
import frc.robot.commands.ResetIMU;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.IntakeSubsystem;
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


  /**
   * Woojins Drive: 117.4, 0.054, 91.4, 0.76, 0.1, 0.01
   * Woojins Rotate: 120.1, 0.145, 92.2, 0.362, 0.035, 0.03
   */
  public BezierCurve driveBezier = new BezierCurve("drive", 151.0, 0.137, 116, 0.646, 0.1, 0.01);
  public BezierCurve rotateBezier = new BezierCurve("rotate", 164.8, 0.093, 100, 0.84, 0.035, 0.03);

  private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private static final double scalarFactorForSpeed = 0.3;

  public static double maxSpeed = DEFAULT_MAX_SPEED * 1;
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
    autoChooser.addOption("FO - Left 2 Sweep", drivetrain.getAutoPath("FO - 2 Sweeps", true));
    autoChooser.addOption("FO - Right 2 Sweep", drivetrain.getAutoPath("FO - 2 Sweeps", false));
    autoChooser.addOption("P - Bottom Start", drivetrain.getAutoPath("P - Bottom Start", false));
    autoChooser.addOption("P - Top Start", drivetrain.getAutoPath("P - Top Start", false));
    autoChooser.addOption("O - Climb", drivetrain.getAutoPath("Outpost", false));

    autoChooser.addOption("Climb Test", drivetrain.getAutoPath("Climb Only", false));

    autoChooser.addOption("Spin Test", drivetrain.getAutoPath("Spin Testing", false));
    autoChooser.addOption("Slip Test", drivetrain.getAutoPath("Slip Testing", false));
    // autoChooser.addOption("Drive Test", drivetrain.getAutoPath("Drive Straight", false));
    // autoChooser.addOption("Drive Test S", drivetrain.getAutoPath("Drive S", false));
    // autoChooser.addOption("Drive And Return", drivetrain.getAutoPath("Drive And Return", false));
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

    DRIVER.leftTrigger().onTrue(
        new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * scalarFactorForSpeed)
    ).onFalse(
      new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * 1)
    );


    DRIVER.rightTrigger().onTrue(
      CoordinationSubsystem.getInstance().setOverrideCommand(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setOverrideCommand(false)
    );

    DRIVER.start().onTrue(new ResetIMU(drivetrain));

    DRIVER.back().onTrue(
      new InstantCommand(() -> {IntakeSubsystem.getInstance().zeroEncoders(); ShooterSubsystem.getInstance().zeroEncoder();}).ignoringDisable(true)
    );

    /* ==================== OPERATOR KEYBINDS ==================== */

    OPERATOR.leftTrigger().onTrue(
      CoordinationSubsystem.getInstance().setRejectingCommand(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setRejectingCommand(false)
    );

    OPERATOR.rightTrigger().onTrue(
      CoordinationSubsystem.getInstance().setFastTransferCommand(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setFastTransferCommand(false)
    );

    OPERATOR.leftBumper().onTrue(
      CoordinationSubsystem.getInstance().setIntakeCommand()
    );

    OPERATOR.rightBumper().onTrue(
      CoordinationSubsystem.getInstance().setFireCommand()
    );
    
    OPERATOR.rightStick().onTrue(
      CoordinationSubsystem.getInstance().setForceTrenchCommand()
    );

    OPERATOR.x().onTrue(new InstantCommand(() -> TurretSubsystem.getInstance().increaseManualAdjust()));
    OPERATOR.y().onTrue(new InstantCommand(() -> TurretSubsystem.getInstance().resetManualAdjust()));
    OPERATOR.b().onTrue(new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust()));

    OPERATOR.povUp().onTrue(ClimberSubsystem.getInstance().setClimbStateAndLevelCommand());

    OPERATOR.povDown().onTrue(
      VisionSubsystem.getInstance().setOperatorOverrideVisionTrustCommand(true)
    ).onFalse(
      VisionSubsystem.getInstance().setOperatorOverrideVisionTrustCommand(false)
    );

    // OPERATOR.povRight().onTrue(new InstantCommand(() -> VisionSubsystem.getInstance().setFerryTarget("bottom")));
    OPERATOR.povRight().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L1)));
    OPERATOR.povLeft().onTrue(new InstantCommand(() -> VisionSubsystem.getInstance().setFerryTarget("top")));

    OPERATOR.back().onTrue(ClimberSubsystem.getInstance().setForceClimberUpCommand());
    OPERATOR.start().onTrue(ClimberSubsystem.getInstance().setForceClimberDownCommand());

    /* ================ TEST ONLY KEYBINDS ============== */

    DRIVER.leftStick().whileTrue(
        Commands.either(new InstantCommand(), new PositionAlignCommand(drivetrain, AlignPos.LEFT), (() -> DriverStation.isTest()))
      );
    DRIVER.rightStick().whileTrue(
      Commands.either(new InstantCommand(), new PositionAlignCommand(drivetrain, AlignPos.RIGHT), (() -> DriverStation.isTest()))
    );

    DRIVER.leftStick().onTrue(
      Commands.either(CoordinationSubsystem.getInstance().setRejectingCommand(true), new InstantCommand(), (() -> DriverStation.isTest()))
    ).onFalse(
      Commands.either(CoordinationSubsystem.getInstance().setRejectingCommand(false), new InstantCommand(), (() -> DriverStation.isTest()))
    );

    DRIVER.rightStick().onTrue(
      Commands.either(CoordinationSubsystem.getInstance().setFireCommand(), new InstantCommand(), (() -> DriverStation.isTest()))
    );
    
    /* ============================== SYSID ==================================== */
    // DRIVER.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // DRIVER.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // DRIVER.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // DRIVER.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      
    /* =============================== DISABLED KEYBINDS ======================== */

    
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

    
    // OPERATOR.povLeft().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.INIT))));
    // OPERATOR.povUp().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.HANDOFF))));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.END))));

    
    // OPERATOR.b().onTrue(climbSub.GoToNextLevel());
    // OPERATOR.povLeft().onTrue(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.INIT))));
    // OPERATOR.povDown().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.END)));

  }

  public void registeredCommands() {
    // Intake Commands
    NamedCommands.registerCommand("SetIntake", CoordinationSubsystem.getInstance().setIntakeCommand());
    NamedCommands.registerCommand("SetFire", CoordinationSubsystem.getInstance().setFireCommand());
    NamedCommands.registerCommand("SetFastTransfer", CoordinationSubsystem.getInstance().setFastTransferCommand());
    NamedCommands.registerCommand("SetHardTrenchOverrideTrue", CoordinationSubsystem.getInstance().setHardTrenchOverride(true));
    NamedCommands.registerCommand("SetHardTrenchOverrideFalse", CoordinationSubsystem.getInstance().setHardTrenchOverride(false));

    NamedCommands.registerCommand("ClimberUp", CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB).alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new WaitCommand(0.15)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.INIT)))));
    NamedCommands.registerCommand("ClimberDown", new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.END))));

    NamedCommands.registerCommand("ForceTrench", CoordinationSubsystem.getInstance().setForceTrenchCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
