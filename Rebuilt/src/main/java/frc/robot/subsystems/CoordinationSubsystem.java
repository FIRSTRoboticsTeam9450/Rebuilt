package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionSubsystem.ZoneState;

public class CoordinationSubsystem extends SubsystemBase {

    private static CoordinationSubsystem INSTANCE;

    private TurretSubsystem turretSub = TurretSubsystem.getInstance();
    private ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
    private VisionSubsystem visionSub = VisionSubsystem.getInstance();

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

    public double getVelocityTurrent(double time) {
        Translation2d currentTarget = visionSub.getAdjustedCurrentTarget();
        Translation2d futureTarget = visionSub.getAdjustedFutureTarget(time);
        // return (turretSub.getTargetAngle(currentTarget) - turretSub.getTargetAngle(futureTarget))/time;
        return -1; //TODO: make your method ryan >:)
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new CoordinationSubsystem();
        return INSTANCE;
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
