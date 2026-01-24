package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.VisionSubsystem.ZoneState;

public class CoordinationSubsystem extends SubsystemBase {

    private static CoordinationSubsystem INSTANCE;

    private TurretSubsystem turretSub; // = TurretSubsystem.getInstance();
    private ShooterSubsystem shooterSub;// = ShooterSubsystem.getInstance();
    private VisionSubsystem visionSub;// = VisionSubsystem.getInstance();
    private CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

    public enum AbsoluteStates {
        STORING,
        IDLING,
        PREPARING_FOR_SHOOTING,
        SHOOTING,
        TRENCH
    }

    private AbsoluteStates currentState = AbsoluteStates.STORING;
    private AbsoluteStates previousState = currentState;

    private boolean driverAllowingShooting = false;

    public CoordinationSubsystem() {
        
    }
    

    public boolean isShooterReady() {
        return shooterSub.isShooterReady();
    }

    public boolean isTurretReady() {
        return turretSub.isAtGoal();
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

    public ZoneState getCurrentZone() {
        return visionSub.getCurrentZone();
    }

    public Command setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin location) {
        return new GoToPosCommand(trenchDeterminer(location));
    }

    private Pose2d trenchDeterminer(TrenchLocationWRTBlueOrigin location) {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d targetPose = currentPose;
        Translation2d tempTrans;
        if (FieldConstants.isBlueAlliance) {
            switch (location) {
                case CLOSE_LEFT:
                    tempTrans = currentPose.getX() > 4.614 ? TrenchManeuvering.trenchOne : TrenchManeuvering.trenchTwo;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;

                case CLOSE_RIGHT: 
                    tempTrans = currentPose.getX() > 4.614 ? TrenchManeuvering.trenchThree : TrenchManeuvering.trenchFour;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;

                case FAR_LEFT:
                    tempTrans = currentPose.getX() > 11.914 ? TrenchManeuvering.trenchFive : TrenchManeuvering.trenchSix;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;

                case FAR_RIGHT:
                    tempTrans = currentPose.getX() > 11.914 ? TrenchManeuvering.trenchSeven : TrenchManeuvering.trenchEight;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;
            }
        } else {
            switch (location) {
                case CLOSE_LEFT:
                    tempTrans = currentPose.getX() > 4.614 ? TrenchManeuvering.trenchEight : TrenchManeuvering.trenchSeven;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;

                case CLOSE_RIGHT: 
                    tempTrans = currentPose.getX() > 4.614 ? TrenchManeuvering.trenchSix : TrenchManeuvering.trenchFive;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;

                case FAR_LEFT:
                    tempTrans = currentPose.getX() > 11.914 ? TrenchManeuvering.trenchFour : TrenchManeuvering.trenchThree;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;

                case FAR_RIGHT:
                    tempTrans = currentPose.getX() > 11.914 ? TrenchManeuvering.trenchTwo : TrenchManeuvering.trenchOne;
                    targetPose = new Pose2d(tempTrans.getX(), tempTrans.getY(), currentPose.getRotation());
                    break;
            }
        }

        return targetPose;
    }

    public double getVelocityTurrent(double time) {
        Translation2d currentTarget = visionSub.getAdjustedCurrentTarget();
        Translation2d futureTarget = visionSub.getAdjustedFutureTarget(time);
        return (turretSub.updateTargetFromMode(currentTarget) - turretSub.updateTargetFromMode(futureTarget))/time;
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CoordinationSubsystem();
            INSTANCE.initizalize();
        }
        return INSTANCE;
    }

    public void initizalize() {
        turretSub = TurretSubsystem.getInstance();
        shooterSub = ShooterSubsystem.getInstance();
        visionSub = VisionSubsystem.getInstance();

    }

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
