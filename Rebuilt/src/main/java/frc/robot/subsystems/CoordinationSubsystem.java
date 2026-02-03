package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.TrenchManeuvering;
import frc.robot.Constants.FieldConstants.TrenchManeuvering.TrenchLocationWRTBlueOrigin;
import frc.robot.commands.GoToPosCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem.TransferState;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import frc.robot.subsystems.VisionSubsystem.ZoneState;

public class CoordinationSubsystem extends SubsystemBase {

    private static volatile CoordinationSubsystem INSTANCE;

    private CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

    public enum AbsoluteStates {
        INIT,
        STORING,
        PREPARING_FOR_SHOOTING,
        SHOOTING,
        TRENCH
    }

    private AbsoluteStates currentState = AbsoluteStates.INIT;
    private ZoneState previousZone = ZoneState.SHOOTING_ZONE;
    private ZoneState currentZone = ZoneState.SHOOTING_ZONE;
    private boolean driverAllowingShooting = false;
    private boolean overrideTrench = false;

    private boolean update = false;
    private boolean previousGoodToShoot = goodToFire();

    public CoordinationSubsystem() {
        
    }
    

    @Override
    public void periodic() {
        currentZone = getCurrentZone();
        getVelocityTurrent(0.02);
        applyZone();
        publishLogs();
    }

    private void applyZone() {
        if(currentZone != previousZone || update || previousGoodToShoot != goodToFire()) {
            if(currentState != AbsoluteStates.INIT) {

                previousZone = currentZone;
                update = false;
                previousGoodToShoot = goodToFire();

                switch (currentZone) {
                    case SHOOTING_ZONE:
                        if(goodToFire()) {
                            setState(AbsoluteStates.SHOOTING);
                        }
                        else {
                            setState(AbsoluteStates.PREPARING_FOR_SHOOTING);
                        }
                        break;
                        
                    case FERRY_ZONE_TOP:
                        if(goodToFire()) {
                            setState(AbsoluteStates.SHOOTING);
                        }
                        else {
                            setState(AbsoluteStates.PREPARING_FOR_SHOOTING);
                        }
                        break; 
                        
                    case FERRY_ZONE_BOTTOM:
                        if(goodToFire()) {
                            setState(AbsoluteStates.SHOOTING);
                        }
                        else {
                            setState(AbsoluteStates.PREPARING_FOR_SHOOTING);
                        }
                        break;
                    case SOFT_TRENCH_DEADZONE:
                        if(!overrideTrench) {
                            setState(AbsoluteStates.TRENCH);
                        }
                        else {
                            setState(AbsoluteStates.PREPARING_FOR_SHOOTING);
                        }
                        break;
                    case HARD_TRENCH_DEADZONE:
                        setState(AbsoluteStates.TRENCH);
                        break;
                    case BACK_DEADZONE:
                        break;
                    default:
                        return;
                }
            }
        }
    }

    private void applyState() {
        TurretState turretState = TurretState.DISABLED;
        ShooterState shooterVelocState = ShooterState.NOT_SHOOTING;
        AngleState shooterHoodState = AngleState.DISABLED;
        TransferState transferState = TransferState.NO_RUN;
        IntakePos intakeAngleState = IntakePos.STORE;
        IntakeState intakeVelocState = IntakeState.IDLING;
        LEDState ledState = LEDState.DISABLED;

        switch (currentState) {
            case INIT: //TODO: Intake should be added when it has zeroing functionalilty, and a hardstop within robot frame.
                turretState = TurretState.INIT;
                shooterHoodState = AngleState.ZEROING;
                ledState = LEDState.INIT;
                break;

            case STORING:
                turretState = TurretState.STORING;
                shooterHoodState = AngleState.STORING;
                ledState = LEDState.STORING;
                break;

            case PREPARING_FOR_SHOOTING:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                transferState = TransferState.HOLD;
                ledState = LEDState.PREPARING_TO_FIRE;
                break;
            case SHOOTING:
                turretState = TurretState.AIMING;
                shooterHoodState = AngleState.AIMING;
                shooterVelocState = ShooterState.SHOOTING;
                transferState = TransferState.RUN_IN;
                ledState = LEDState.GOOD_TO_FIRE;
                break;
            case TRENCH:
                turretState = TurretState.STORING;
                shooterHoodState = AngleState.STORING;
                shooterVelocState = ShooterState.NOT_SHOOTING;
                transferState = TransferState.HOLD;
                ledState = LEDState.UNABLE_TO_FIRE;
                break;


        }

        TurretSubsystem.getInstance().setState(turretState);
        ShooterSubsystem.getInstance().setState(shooterVelocState, shooterHoodState);
        TransferSubsystem.getInstance().setState(transferState);
        IntakeSubsystem.getInstance().setPos(intakeAngleState);
        IntakeSubsystem.getInstance().setState(intakeVelocState);
        LEDSubsystem.getInstance().setState(ledState);
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Coord/Current State", currentState);
    }
    // ======================== Conditions =============================

    public boolean isShooterReady() {
        return ShooterSubsystem.getInstance().isShooterReady();
    }

    public boolean isTurretReady() {
        return TurretSubsystem.getInstance().isAtGoal();
    }

    public boolean areShooterAndTurretReady() {
        return (isShooterReady() && isTurretReady());
    }

    public boolean isDriverWantingToShoot() {
        return driverAllowingShooting;
    }

    public boolean goodToFire() {
        return (driverAllowingShooting && areShooterAndTurretReady());
    }

    public Command setFireCommand(boolean value) {
        return new InstantCommand(() -> driverAllowingShooting = value);
    }

    public Command setOverrideCommand(boolean value) {
        return new InstantCommand(() -> overrideTrench = value)
        .alongWith(new InstantCommand(() -> update = true));
        
    }

    // ============================ Getters & Setters ============================

    public ZoneState getCurrentZone() {
        return VisionSubsystem.getInstance().getCurrentZone();
    }

    public Command setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin location) {
        switch (location) {
            case CLOSE_LEFT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchOne, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchTwo, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchEight, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSeven, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            (() -> FieldConstants.isBlueAlliance)
                        );


            case CLOSE_RIGHT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchThree, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFour, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSix, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFive, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            (() -> FieldConstants.isBlueAlliance)
                        );

            case FAR_LEFT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFive, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSix, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchFour, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchThree, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            (() -> FieldConstants.isBlueAlliance)
                        );

            case FAR_RIGHT:
                return Commands.either(
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchSeven, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchEight, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 11.914)),
                            Commands.either(
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchTwo, new Rotation2d(0))),
                                new GoToPosCommand(new Pose2d(TrenchManeuvering.trenchOne, new Rotation2d(0))),
                                (() -> drivetrain.getState().Pose.getX() > 4.614)),
                            (() -> FieldConstants.isBlueAlliance)
                        );
        }

    return new GoToPosCommand(drivetrain.getState().Pose);
    }
    public boolean getInDeadzone() {
        if(currentZone != ZoneState.SOFT_TRENCH_DEADZONE || currentZone != ZoneState.HARD_TRENCH_DEADZONE || currentZone != ZoneState.BACK_DEADZONE) {
            return true;
        }
        return false;
    }
    public boolean getOverridinTrench() {
        if(overrideTrench) {
            return true;
        }
        return false;
    }
    public double getVelocityTurrent(double time) {
        Translation2d currentTarget = VisionSubsystem.getInstance().getAdjustedCurrentTarget();
        Translation2d futureTarget = VisionSubsystem.getInstance().getAdjustedFutureTarget(time);
        
        Logger.recordOutput("Rebuilt/Turret/FutureTurretAngle", TurretSubsystem.getInstance().updateTargetFromMode(futureTarget));
        Logger.recordOutput("Rebuilt/Turret/CurrentTurretAngle", TurretSubsystem.getInstance().updateTargetFromMode(currentTarget));
        return (TurretSubsystem.getInstance().updateTargetFromMode(futureTarget) - TurretSubsystem.getInstance().updateTargetFromMode(currentTarget))/time;
    }

    public void setState(AbsoluteStates state) {
        currentState = state;
        applyState();
    }

    // ======================= INIT ===================

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CoordinationSubsystem();
            INSTANCE.initizalize();
        }
        return INSTANCE;
    }

    public void initizalize() {
    }

    // ========================================= GAME DATA ===========================================

    public static class GameData {
        private String gameData;
        private boolean isBlueHubActive = true;

        public void updateData() {
            gameData = DriverStation.getGameSpecificMessage();
            if (gameData.length() > 0) {
                if (gameData.charAt(0) == 'B') {
                    isBlueHubActive = true;
                } else {
                    isBlueHubActive = false;
                }
            }
        }

        public boolean isBlueHubActive() {
            return isBlueHubActive;
        }
    }
}
