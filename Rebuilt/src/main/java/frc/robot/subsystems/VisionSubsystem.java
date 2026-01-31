package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants.TurretConstants;
import frc.robot.Constants.RobotConstants.LimelightConstants.BackLeft;
import frc.robot.Constants.RobotConstants.LimelightConstants.BackRight;
import frc.robot.Constants.RobotConstants.LimelightConstants.FrontLeft;
import frc.robot.Constants.RobotConstants.LimelightConstants.FrontRight;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneConstantsAndyMark;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneConstantsWelded;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneTargets;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightObject;
import frc.robot.util.LimelightObject.Location;

import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem using Limelights with:
 *  - Fusion of multiple cameras
 *  - Singleton pattern
 *  - Integration with drivetrain for pose estimation
 *
 * This subsystem reads Limelight data, fuses multiple camera poses,
 * and provides utility functions like distance to target and shooter recommendations.
 * 
 * To-Do:
 *  - Make work with any amount of limelight
 *  - Use tolerances to determine if a vision pose is valid (look at if value is off by a tolerance. (except x and y (Pose3d)))
 *  - Lessen the Standard deviation for rotation, is to high. Good ballpark: 999
 *  - Other std deviations could also be lessened
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------
    // Singleton instance
    // -------------------------
    private static volatile VisionSubsystem instance;

    /**
     * Singleton accessor method.
     * Ensures only one instance exists and is globally accessible.
     *
     * @param drivetrain Drivetrain instance for updating pose with vision
     * @return VisionSubsystem instance
     */
    public static synchronized VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
            instance.initizalize();
        }
        return instance;
    }

    public void initizalize() {
    }

    // -------------------------
    // Core subsystem fields
    // -------------------------
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain; // Reference to drivetrain for pose updates

    private volatile Pose2d lastVisionPose = drivetrain.getState().Pose;  // Last pose estimated from fused vision
    private volatile double lastVisionTimestamp = 0.0; // Timestamp of last vision update

    // Standard deviations for vision measurements (for filtering/fusion)
    private double visionStdDevX = 0.5;
    private double visionStdDevY = 0.5;
    private double visionStdDevTheta = 999.0;

    // Maximum allowed differences for fusion; if exceeded, fallback to higher confidence camera
    private double maxTranslationDiff = 2.5; // meters
    private double maxRotationDiff = Math.toRadians(45); // radians

    // Validation thresholds for pose filtering
    private static final double MAX_Z_THRESHOLD = 1; // Robot must be on ground
    private static final double MAX_PITCH_THRESHOLD = 1000; // Robot must not be tilted forward/back
    private static final double MAX_ROLL_THRESHOLD = 1000; // Robot must not be tilted side-to-side
    private static final double MAX_YAW_DIFF_DEGREES = 2.5; // Vision yaw must match Pigeon within 0.01 degrees

    // Limelight camera objects
    private final List<LimelightObject> LIMELIGHT_LIST = new ArrayList<>();

    // Logging
    private volatile Pose2d lastFusedPose = null;   // Last fused pose from vision
    private volatile Translation2d lastTargetPose = null;
    private volatile Translation2d lastAdjustedTargetPose = null;

    // Targeting Data
    public enum ZoneState {
        SHOOTING_ZONE,
        FERRY_ZONE_TOP,
        FERRY_ZONE_BOTTOM,
        TRENCH_DEADZONE,
        BACK_DEADZONE
    }
    ZoneState currentZone = ZoneState.SHOOTING_ZONE;
    Translation2d currentTarget = ZoneTargets.SHOOTING_TARGET;

    // -------------------------
    // Constructor (private for singleton)
    // -------------------------
    @SuppressWarnings("static-access")
    private VisionSubsystem() {
        drivetrain = RobotContainer.drivetrain;

        // Configure camera offsets and add to internal list
        configureLimelights();

        // Initialize drivetrain vision standard deviations
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta));
    }

    /**
     * Initializes LimelightObject instances with physical offsets and names.
     * Ensures that all camera poses are correctly transformed to robot frame.
     */
    private void configureLimelights() {
        // Front Left Limelight
        LimelightObject limelightFL = new LimelightObject(FrontLeft.NAME);
        limelightFL.location = Location.FRONT_LEFT;
        limelightFL.offsetX = FrontLeft.LOCATION_X;
        limelightFL.offsetY = FrontLeft.LOCATION_Y;
        limelightFL.offsetZ = FrontLeft.LOCATION_Z;
        limelightFL.offsetPitch = FrontLeft.PITCH;
        limelightFL.offsetRoll = FrontLeft.ROLL;
        limelightFL.offsetYaw = FrontLeft.YAW;
        LIMELIGHT_LIST.add(limelightFL);

        // Front Right Limelight
        LimelightObject limelightFR = new LimelightObject(FrontRight.NAME);
        limelightFR.location = Location.FRONT_RIGHT;
        limelightFR.offsetX = FrontRight.LOCATION_X;
        limelightFR.offsetY = FrontRight.LOCATION_Y;
        limelightFR.offsetZ = FrontRight.LOCATION_Z;
        limelightFR.offsetPitch = FrontRight.PITCH;
        limelightFR.offsetRoll = FrontRight.ROLL;
        limelightFR.offsetYaw = FrontRight.YAW;
        LIMELIGHT_LIST.add(limelightFR);

        // Back Left Limelight
        LimelightObject limelightBL = new LimelightObject(BackLeft.NAME);
        limelightBL.location = Location.BACK_LEFT;
        limelightBL.offsetX = BackLeft.LOCATION_X;
        limelightBL.offsetY = BackLeft.LOCATION_Y;
        limelightBL.offsetZ = BackLeft.LOCATION_Z;
        limelightBL.offsetPitch = BackLeft.PITCH;
        limelightBL.offsetRoll = BackLeft.ROLL;
        limelightBL.offsetYaw = BackLeft.YAW;
        LIMELIGHT_LIST.add(limelightBL);

        // Back Right Limelight
        LimelightObject limelightBR = new LimelightObject(BackRight.NAME);
        limelightBR.location = Location.BACK_RIGHT;
        limelightBR.offsetX = BackRight.LOCATION_X;
        limelightBR.offsetY = BackRight.LOCATION_Y;
        limelightBR.offsetZ = BackRight.LOCATION_Z;
        limelightBR.offsetPitch = BackRight.PITCH;
        limelightBR.offsetRoll = BackRight.ROLL;
        limelightBR.offsetYaw = BackRight.YAW;
        LIMELIGHT_LIST.add(limelightBR);
    }

    /**
     * Periodic method called every ~20ms by the scheduler.
     * Updates vision data and logs key metrics.
     */
    @Override
    public void periodic() {
        updateTargetAndZone();
        updateVision();  // Compute fused vision pose
        publishLogs();   // Print/log fusion confidence and pose
    }

    // -------------------------
    // Vision update + fusion
    // -------------------------

    /**
     * Updates pose estimates from all Limelight cameras and fuses them.
     * Sends fused pose to drivetrain for odometry correction.
     */
    private void updateVision() {
        double robotYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble() - RobotContainer.pigeonOffset;
        
        // Collect vision measurements from all limelights
        List<VisionMeasurement> measurements = new ArrayList<>();
        
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            // Update robot orientation for this limelight
            LimelightHelpers.SetRobotOrientation(limelight.getName(), robotYaw, 0, 0, 0, 0, 0);
            
            // Get pose estimate using MegaTag2
            LimelightHelpers.PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
            
            // Create vision measurement
            VisionMeasurement vm = createVisionMeasurement(pe, limelight.getName());
            
            // Only add valid measurements
            if (vm.isValid()) {
                measurements.add(vm);
            }
        }
        
        // Fuse all valid measurements
        Optional<Pose2d> fusedPose = fuseVision(measurements);
        
        // If valid, update lastVisionPose and send to drivetrain
        fusedPose.ifPresent(pose -> {
            lastVisionPose = pose;
            lastFusedPose = pose;
            
            // Use the latest timestamp from all measurements
            lastVisionTimestamp = measurements.stream()
                .mapToDouble(VisionMeasurement::timestamp)
                .max()
                .orElse(0.0);
            
            // Set standard deviations and add measurement
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta));
            drivetrain.addVisionMeasurement(pose, Utils.currentTimeToFPGATime(lastVisionTimestamp));
            
            Logger.recordOutput("Rebuilt/Vision/ValidCameraCount", measurements.size());
        });
    }

    /**
     * Converts a Limelight PoseEstimate into a VisionMeasurement.
     * Includes confidence calculation and validity check with pose filtering.
     * 
     * Pose is considered invalid if:
     * - Z position is not zero (robot airborne or on terrain)
     * - Pitch rotation is not zero (robot tilting forward/backward)
     * - Roll rotation is not zero (robot tilting side-to-side)
     * - Yaw differs from Pigeon IMU by more than 0.01 degrees
     */
    private VisionMeasurement createVisionMeasurement(LimelightHelpers.PoseEstimate pe, String limelightName) {
        if (pe == null || pe.pose == null) return VisionMeasurement.invalid();

        double ambiguity = computeAmbiguity(limelightName);
        double timestamp = Utils.fpgaToCurrentTime(pe.timestampSeconds);
        Pose2d pose = (Pose2d) pe.pose;
        int tagCount = pe.tagCount;

        // Get Pose3d data for validation
        // We need to get the raw 3D pose for validation
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        
        boolean poseValid = true;
        
        if (results != null) {
            // Extract 3D pose data from botpose array
            // botpose format: [x, y, z, roll, pitch, yaw, ...]
            double[] botpose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
            
            if (botpose != null && botpose.length >= 6) {
                double z = botpose[2];
                double roll = botpose[3];
                double pitch = botpose[4];
                double visionYaw = botpose[5];
                
                // Get current Pigeon yaw
                double pigeonYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble() - RobotContainer.pigeonOffset;
                
                // Validation checks
                boolean zValid = Math.abs(z) <= MAX_Z_THRESHOLD;
                boolean rollValid = Math.abs(roll) <= MAX_ROLL_THRESHOLD;
                boolean pitchValid = Math.abs(pitch) <= MAX_PITCH_THRESHOLD;
                
                // Yaw difference check (handle angle wrapping)
                double yawDiff = Math.abs(visionYaw - pigeonYaw);
                // Normalize to [-180, 180]
                while (yawDiff > 180) yawDiff -= 360;
                while (yawDiff < -180) yawDiff += 360;
                boolean yawValid = Math.abs(yawDiff) <= MAX_YAW_DIFF_DEGREES;
                
                // Pose is only valid if all checks pass
                poseValid = zValid && rollValid && pitchValid && yawValid;
                
                // Optional: Log which check failed for debugging
                if (!poseValid) {
                    if (!zValid) Logger.recordOutput("Rebuilt/Vision/" + limelightName + "/InvalidZ", z);
                    if (!rollValid) Logger.recordOutput("Rebuilt/Vision/" + limelightName + "/InvalidRoll", roll);
                    if (!pitchValid) Logger.recordOutput("Rebuilt/Vision/" + limelightName + "/InvalidPitch", pitch);
                    if (!yawValid) Logger.recordOutput("Rebuilt/Vision/" + limelightName + "/InvalidYawDiff", yawDiff);
                }
            }
        }

        // Consider measurement valid if at least 1 tag, low ambiguity, AND pose validation passes
        boolean valid = tagCount > 0 && poseValid;

        // Confidence weighted: 70% ambiguity, 30% tag count
        double tagFactor = Math.min(tagCount / 2.0, 1.0);
        double confidence = (1.0 - ambiguity) * 0.7 + tagFactor * 0.3;

        return new VisionMeasurement(pose, confidence, timestamp, valid);
    }

    /**
     * Computes the best (lowest) ambiguity among all fiducials seen by a Limelight.
     * @Returns 1.0 if no fiducials detected (completely ambiguous).
     */
    private double computeAmbiguity(String limelightName) {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        if (fiducials == null || fiducials.length == 0) return 1.0;

        double best = 1.0;
        for (LimelightHelpers.RawFiducial f : fiducials) {
            if (f == null) continue;
            best = Math.min(best, f.ambiguity);
        }
        return best;
    }

    /**
     * Fuses multiple VisionMeasurements into a single Pose2d.
     * 
     * Algorithm:
     * 1. If no valid measurements, return empty
     * 2. If only one valid measurement, return it
     * 3. If multiple measurements:
     *    a. Compute mean pose
     *    b. Reject outliers that differ too much from mean
     *    c. Fuse remaining measurements using confidence weighting
     * 
     * @param measurements List of vision measurements to fuse
     * @return Optional containing fused pose, or empty if no valid measurements
     */
    private Optional<Pose2d> fuseVision(List<VisionMeasurement> measurements) {
        // Filter to only valid measurements
        List<VisionMeasurement> validMeasurements = measurements.stream()
            .filter(VisionMeasurement::isValid)
            .toList();
        
        // No valid measurements
        if (validMeasurements.isEmpty()) {
            return Optional.empty();
        }
        
        // Single valid measurement - use it directly
        if (validMeasurements.size() == 1) {
            return Optional.of(validMeasurements.get(0).pose());
        }
        
        // Multiple measurements - compute mean for outlier detection
        double meanX = validMeasurements.stream()
            .mapToDouble(m -> m.pose().getX())
            .average()
            .orElse(0.0);
        
        double meanY = validMeasurements.stream()
            .mapToDouble(m -> m.pose().getY())
            .average()
            .orElse(0.0);

        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //TODO: Actually average together
        // double meanRotation = validMeasurements.stream()
        //     .mapToDouble(m -> m.pose().getRotation().getRadians())
        //     .average()
        //     .orElse(0.0);

        double meanRotation = validMeasurements.get(0).pose.getRotation().getRadians();
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        Pose2d meanPose = new Pose2d(meanX, meanY, new Rotation2d(meanRotation));
        
        // Filter out outliers that differ too much from mean
        List<VisionMeasurement> inliers = validMeasurements.stream()
            .filter(m -> {
                double translationDiff = m.pose().getTranslation().getDistance(meanPose.getTranslation());
                double rotationDiff = Math.abs(m.pose().getRotation().minus(meanPose.getRotation()).getRadians());
                
                return translationDiff < maxTranslationDiff && rotationDiff < maxRotationDiff;
            })
            .toList();
        
        // If all measurements were outliers, fall back to highest confidence measurement
        if (inliers.isEmpty()) {
            return validMeasurements.stream()
                .max((m1, m2) -> Double.compare(m1.confidence(), m2.confidence()))
                .map(VisionMeasurement::pose);
        }
        
        // If only one inlier remains, use it
        if (inliers.size() == 1) {
            return Optional.of(inliers.get(0).pose());
        }
        // TODO: Log number of outliers
        
        // Fuse inliers using confidence-weighted average
        double totalConfidence = inliers.stream()
            .mapToDouble(VisionMeasurement::confidence)
            .sum();
        
        double fusedX = inliers.stream()
            .mapToDouble(m -> m.pose().getX() * m.confidence())
            .sum() / totalConfidence;
        
        double fusedY = inliers.stream()
            .mapToDouble(m -> m.pose().getY() * m.confidence())
            .sum() / totalConfidence;
        
        // double fusedRotation = inliers.stream()
        //     .mapToDouble(m -> m.pose().getRotation().getRadians() * m.confidence())
        //     .sum() / totalConfidence;
        double fusedRotation = inliers.get(0).pose().getRotation().getRadians();
        return Optional.of(new Pose2d(fusedX, fusedY, new Rotation2d(fusedRotation)));
    }

    // -------------------------
    // Logging
    // -------------------------
    private void publishLogs() {
        // Overall vision state
        Logger.recordOutput("Rebuilt/Vision/Total/Last Vision Pose", lastVisionPose);
        Logger.recordOutput("Rebuilt/Vision/Total/Last Fused Pose", lastFusedPose);
        Logger.recordOutput("Rebuilt/Vision/Total/Last Target Pose", lastTargetPose);
        Logger.recordOutput("Rebuilt/Vision/Total/Last Vision Timestamp", lastVisionTimestamp);
        
        // Standard deviations
        Logger.recordOutput("Rebuilt/Vision/Config/StdDev X", visionStdDevX);
        Logger.recordOutput("Rebuilt/Vision/Config/StdDev Y", visionStdDevY);
        Logger.recordOutput("Rebuilt/Vision/Config/StdDev Theta", visionStdDevTheta);
        
        // Fusion thresholds
        Logger.recordOutput("Rebuilt/Vision/Config/Max Translation Diff", maxTranslationDiff);
        Logger.recordOutput("Rebuilt/Vision/Config/Max Rotation Diff", Math.toDegrees(maxRotationDiff));
        
        // Per-camera data
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            String name = limelight.getName();
            LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
            
            if (results != null) {
                Logger.recordOutput("Rebuilt/Vision/" + name + "/Valid", results.valid);
                Logger.recordOutput("Rebuilt/Vision/" + name + "/Priority Tag", LimelightHelpers.getFiducialID(name));
                Logger.recordOutput("Rebuilt/Vision/" + name + "/Pipeline Latency", results.latency_pipeline);
                Logger.recordOutput("Rebuilt/Vision/" + name + "/Capture Latency", results.latency_capture);
                
                // Get botpose data if available
                double[] botpose = LimelightHelpers.getBotPose_wpiBlue(name);
                if (botpose != null && botpose.length >= 6) {
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/X", botpose[0]);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Y", botpose[1]);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Z", botpose[2]);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Roll", botpose[3]);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Pitch", botpose[4]);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Yaw", botpose[5]);
                }
                
                // Ambiguity for this camera
                Logger.recordOutput("Rebuilt/Vision/" + name + "/Ambiguity", computeAmbiguity(name));
            }
        }
        
        // Robot state
        Logger.recordOutput("Rebuilt/Vision/Robot/Pigeon Yaw", 
            drivetrain.getPigeon2().getYaw().getValueAsDouble() - RobotContainer.pigeonOffset);
        
        // Fault detection
        Logger.recordOutput("Rebuilt/Vision/Faults/Drivetrain instance null?", drivetrain == null);
        Logger.recordOutput("Rebuilt/Vision/Faults/Limelight List Empty?", LIMELIGHT_LIST.isEmpty());
        Logger.recordOutput("Rebuilt/Vision/Faults/Limelight Count", LIMELIGHT_LIST.size());

        // Current bot pose
        // Logger.recordOutput("Rebuilt/Vision/CurrentPose", drivetrain.getState().Pose);
        Logger.recordOutput("Rebuilt/Vision/CurrentPoseFromMethod", getCurrentBotPoseAllianceUnspecific());
        Logger.recordOutput("Rebuilt/Vision/CurrentPose", drivetrain.getState().Pose);
        Logger.recordOutput("Rebuilt/Vision/Alliance", DriverStation.getAlliance().toString());
        Logger.recordOutput("Rebuilt/Vision/CurrentZone", currentZone);
        Logger.recordOutput("Rebuilt/Vision/Distance From target", getDistanceToTarget(getTargetRelativeToBot()));
        Logger.recordOutput("Rebuilt/Vision/CurrentTargetFromMethod", new Pose2d(getAdjustedCurrentTarget(), new Rotation2d()));
        // Logger.recordOutput("Rebuilt/Vision/FutureTarget", new Pose2d(getAdjustedFutureTarget(.02), new Rotation2d(getCurrentBotPoseAllianceUnspecific().getRotation().getRadians())).minus(getCurrentBotPoseAllianceUnspecific()));
        Logger.recordOutput("Rebuilt/Vision/CurrentTarget", getCurrentBotPoseAllianceUnspecific().plus(new Transform2d(getAdjustedCurrentTarget(), new Rotation2d())));
        Logger.recordOutput("Rebuilt/Vision/CurrentTargetWRTRobot", getTargetRelativeToBot());
        // getAdjustedFutureTarget(.5);
    }

    // -------------------------
    // Public API
    // -------------------------
    public Pose2d getLastFusedPose() { return lastFusedPose; }
    public Pose2d getLastVisionPose() { return lastVisionPose; }

    /**
     * Computes the distance from robot to a target pose.
     */
    public double getDistanceToTarget(Translation2d target) {
        // Pose2d drivePose = getCurrentBotPoseAllianceUnspecific();
        // Transform2d tmpTransform = new Transform2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation());
        // Transform2d currentPose = tmpTransform;
        // if (currentPose == null) return Double.POSITIVE_INFINITY;
        // lastTargetPose = target;
        return Math.sqrt(Math.pow(target.getX(), 2) + Math.pow(target.getY(), 2));
        // return currentPose.getTranslation().getDistance(target);
    }
    
    /**
     * Computes the robot-to-target transform in robot frame.
     * TODO: Rewrite
     */
    public Translation2d getRobotToTarget(Translation2d target) {
        if (lastVisionPose == null) return null;

        lastTargetPose = target;

        Translation2d deltaField = target.minus(lastVisionPose.getTranslation());
        Translation2d translationRobotFrame = deltaField.rotateBy(lastVisionPose.getRotation().unaryMinus());
        // Rotation2d rotationRobotToTarget = target.getRotation().minus(lastVisionPose.getRotation());

        return translationRobotFrame;
    }

    public Pose2d getCurrentBotPoseAllianceUnspecific() {
        Pose2d currentBluePose = drivetrain.getState().Pose;
        Pose2d alliancePose = new Pose2d(ZoneConstantsWelded.RED_CORNER, new Rotation2d(Math.PI));

        if(DriverStation.getAlliance().toString().equals("Optional[Red]")){
            alliancePose = new Pose2d(ZoneConstantsWelded.RED_CORNER, new Rotation2d(Math.PI));
            // Pose2d newBotPose = currentBluePose.plus(redCornerPose);
            // Transform2d calculated = redCorner.minus(currentBluePose);
            // return new Pose2d(calculated.getTranslation(), calculated.getRotation());
        }
        return currentBluePose.relativeTo(alliancePose);
        
    }

    public Translation2d getTargetRelativeToBot () {
        Pose2d currentPose = getCurrentBotPoseAllianceUnspecific().transformBy(TurretConstants.OFFSET_WRT_CENTER_OF_ROBOT);
        Pose2d currentTargetPose = new Pose2d(currentTarget, new Rotation2d());
        return currentTargetPose.minus(currentPose).getTranslation();
    }

    public ChassisSpeeds getVelocityOfBot() {
        ChassisSpeeds speed = drivetrain.getState().Speeds;
        Pose2d currentPose = getCurrentBotPoseAllianceUnspecific();
        return ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, currentPose.getRotation());
    }

    public Translation2d getTargetPose() {
        return lastTargetPose;
    }

    public Translation2d getCurrentTarget() {
        return currentTarget;
    }
    public Translation2d getAdjustCurrentTargetBlueAlliance() {
        Translation2d currentTargetAllianceUnspecific = getAdjustedCurrentTarget();
        Translation2d blueCorner = new Translation2d();
        return currentTargetAllianceUnspecific;

    }
    public Translation2d getAdjustedCurrentTarget() {
        lastAdjustedTargetPose = getAdjustedTarget(getTargetRelativeToBot(), lastAdjustedTargetPose);
        return lastAdjustedTargetPose;
    }
    /**
     * Get the pose of the adjusted target based on how fast it's moving
     * @return
     */
    public Translation2d getAdjustedTarget(Translation2d givenTargetWRTBot, Translation2d lastTargetPose) {
        Translation2d targetWRTBot = givenTargetWRTBot;
        ChassisSpeeds velocityRobotWRTRobot = drivetrain.getState().Speeds;
        // ChassisSpeeds velocityRobotWRTField = ChassisSpeeds.fromRobotRelativeSpeeds(velocityRobotWRTRobot, currentPose.getRotation());
        double timeToShoot = getTimeToShoot(targetWRTBot);
        if(lastTargetPose != null) {
            timeToShoot = getTimeToShoot(lastTargetPose);
        }
        Translation2d amountMoved = new Translation2d(velocityRobotWRTRobot.vxMetersPerSecond * timeToShoot, velocityRobotWRTRobot.vyMetersPerSecond * timeToShoot);
        Translation2d finalTarget = targetWRTBot.plus(amountMoved.times(-1));

        // Iterate to zoom in to find the final target, get closer and closer each time
        // for(int i = 0; i < 6; i++) {
        //     timeToShoot = getTimeToShoot(finalTarget);
        //     amountMoved =  new Translation2d(velocityRobotWRTRobot.vxMetersPerSecond * timeToShoot, velocityRobotWRTRobot.vyMetersPerSecond * timeToShoot);
        //     finalTarget = targetWRTBot.plus(amountMoved.times(-1));
        // }
        
        return finalTarget;
    }

    /**
     * Get the future target's pose in a certain amount of time, use this and the 
     * current target's pose to find velocity
     * @param time
     * @return
     */
    public Translation2d getAdjustedFutureTarget(double time) {
        Pose2d currentPoseWRTField = getCurrentBotPoseAllianceUnspecific();
        ChassisSpeeds velocityRobotWRTRobot = drivetrain.getState().Speeds;
        // ChassisSpeeds velocityRobotWRTField = ChassisSpeeds.fromRobotRelativeSpeeds(velocityRobotWRTRobot, currentPose.getRotation());
        double newRotation = velocityRobotWRTRobot.omegaRadiansPerSecond * time + currentPoseWRTField.getRotation().getRadians();
        // Pose2d futurePose = currentPose.plus(
        //     new Transform2d(new Translation2d((velocityRobotWRTRobot.vxMetersPerSecond * time), (velocityRobotWRTRobot.vyMetersPerSecond * time)),
        //     new Rotation2d(velocityRobotWRTRobot.omegaRadiansPerSecond * time)));
        Pose2d futurePoseWRTCurrentBot = new Pose2d(//new Translation2d(), new Rotation2d());
            new Translation2d((velocityRobotWRTRobot.vxMetersPerSecond * time), (velocityRobotWRTRobot.vyMetersPerSecond * time)),
            // new Rotation2d());
            new Rotation2d(velocityRobotWRTRobot.omegaRadiansPerSecond * time)); // ABSOLUTELY CORRECT
        //Transform2d botToField = fieldToBot.times(-1);

        Pose2d targetWRTCurrentBot = new Pose2d(getTargetRelativeToBot(), new Rotation2d()); // Correct
        Transform2d fieldWRTCurrentTransform = new Transform2d(currentPoseWRTField.getTranslation(), currentPoseWRTField.getRotation()).inverse(); // ABSOLUTELY CORRECT
        Pose2d fieldWRTCurrentPose = new Pose2d(fieldWRTCurrentTransform.getTranslation(), fieldWRTCurrentTransform.getRotation()); // ABSOLUTELY CORRECT
        // Transform2d futurePoseWRTField2 = futurePoseWRTCurrentBot.plus(currentPoseWRTField);
        Pose2d futurePoseWRTField = futurePoseWRTCurrentBot.relativeTo(fieldWRTCurrentPose); // ABSOLUTELY CORRECT
        Transform2d futurePoseToTarget = targetWRTCurrentBot.minus(futurePoseWRTCurrentBot); 
        // Transform2d targetWRTField = futurePoseWRTField.plus(targetWRTFuturePose);
        Transform2d futurePoseToAdjustedTarget = new Transform2d(getAdjustedTarget(futurePoseToTarget.getTranslation(), futurePoseToTarget.getTranslation()), new Rotation2d());
        Logger.recordOutput("Rebuilt/Vision/FuturePose", futurePoseWRTCurrentBot.relativeTo(fieldWRTCurrentPose));
        Logger.recordOutput("Rebuilt/Vision/FutureTarget", futurePoseWRTField.plus(futurePoseToAdjustedTarget));
        return getAdjustedTarget(futurePoseToTarget.getTranslation(), futurePoseToTarget.getTranslation());
        // Pose2d targetWRTCurrentBot = futurePose.plus((new Pose2d(getAdjustedCurrentTarget(), new Rotation2d()).minus(new Pose2d(futurePoseWRTCurrentBot.getTranslation(), futurePoseWRTCurrentBot.getRotation()))));
        // Logger.recordOutput("Rebuilt/Vision/FutureBotPoseWRTCurrentBot", futurePoseWRTField.plus(futurePoseWRTCurrentBot));
        // Logger.recordOutput("Rebuilt/Vision/FutureBotPoseWRTField", futurePoseWRTField);
        // return targetWRTFuturePose.getTranslation();
        // Logger.recordOutput("Rebuilt/Vision/FutureTarget", currentPose.plus(new Transform2d(targetWRTCurrentBot.getTranslation(), targetWRTCurrentBot.getRotation())));
        // return new Pose2d(getAdjustedCurrentTarget(), new Rotation2d()).minus(new Pose2d(futurePoseWRTCurrentBot.getTranslation(), futurePoseWRTCurrentBot.getRotation())).getTranslation();

        //futurePose.relativeTo(new Pose2d(ZoneConstantsAndyMark.RED_CORNER, new Rotation2d()));
        
        // Pose2d currentTargetPose = new Pose2d(currentTarget, new Rotation2d());
        
        // Translation2d futureTargetWRTBot = currentTargetPose.minus(futurePose).getTranslation(); //new Translation2d(-futurePose.getX() + currentTarget.getX(), -futurePose.getY() + currentTarget.getY());//
        // Logger.recordOutput("Rebuilt/Vision/FutureTargetPose WRT Bot", new Pose2d(futureTargetWRTBot, new Rotation2d()));
        
        // Logger.recordOutput("Rebuilt/Vision/FutureTarget", futurePose.plus(new Transform2d(getAdjustedTarget(futureTargetWRTBot, futureTargetWRTBot), new Rotation2d())));
        // return getAdjustedTarget(futureTargetWRTBot, futureTargetWRTBot);


        // return lastAdjustedFutureTargetPose;

    }

    /**
     * Get the air time of the ball to get to its target
     * @param target
     * @return
     */
    public double getTimeToShoot(Translation2d target) {
        double timeToShoot = 1.2;
        // if(currentTarget.equals(ZoneTargets.FERRY_TOP_TARGET) || currentTarget.equals(ZoneTargets.FERRY_BOTTOM_TARGET)) {
        //     timeToShoot = shooterSub.getFerryingDistanceToTime(getDistanceToTarget(target));
        // }
        // else if(currentTarget.equals(ZoneTargets.SHOOTING_TARGET)) {
        //     timeToShoot = shooterSub.getShootingDistanceToTime(getDistanceToTarget(target));
        // }
        return timeToShoot;
    }

    public ZoneState getCurrentZone() {
        return currentZone;
    }

    /**
     * Update which zone it's in and target it's in
     */
    public void updateTargetAndZone() {
        Pose2d currentPose = getCurrentBotPoseAllianceUnspecific();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        if(currentX > ZoneConstantsAndyMark.MAJOR_ZONE_X_LOW && currentX < ZoneConstantsAndyMark.MAJOR_ZONE_X_HIGH && currentY > ZoneConstantsAndyMark.MAJOR_ZONE_Y_LOW && currentY < ZoneConstantsAndyMark.MAJOR_ZONE_Y_HIGH) { // CHANGE DURING COMP TO WELDED
            currentZone = ZoneState.SHOOTING_ZONE;
            currentTarget = ZoneTargets.SHOOTING_TARGET;
        }
        else if(currentX > ZoneConstantsAndyMark.MID_ZONE_X_LOW && currentX < ZoneConstantsAndyMark.MID_ZONE_X_HIGH && currentY > ZoneConstantsAndyMark.MID_ZONE_1_Y_LOW && currentY < ZoneConstantsAndyMark.MID_ZONE_1_Y_HIGH) {
            currentZone = ZoneState.FERRY_ZONE_TOP;
            currentTarget = ZoneTargets.FERRY_TOP_TARGET;
        }
        else if(currentX > ZoneConstantsAndyMark.MID_ZONE_X_LOW && currentX < ZoneConstantsAndyMark.MID_ZONE_X_HIGH && currentY > ZoneConstantsAndyMark.MID_ZONE_2_Y_LOW && currentY < ZoneConstantsAndyMark.MID_ZONE_2_Y_HIGH) {
            currentZone = ZoneState.FERRY_ZONE_BOTTOM;
            currentTarget = ZoneTargets.FERRY_BOTTOM_TARGET;
        }
        if(currentX > ZoneConstantsAndyMark.TRENCH_12_DEADZONE_X_LOW && currentX < ZoneConstantsAndyMark.TRENCH_12_DEADZONE_X_HIGH && currentY > ZoneConstantsAndyMark.TRENCH_1_DEADZONE_Y_LOW && currentY < ZoneConstantsAndyMark.TRENCH_1_DEADZONE_Y_HIGH) {
            currentZone = ZoneState.TRENCH_DEADZONE;
        }
        else if(currentX > ZoneConstantsAndyMark.TRENCH_12_DEADZONE_X_LOW && currentX < ZoneConstantsAndyMark.TRENCH_12_DEADZONE_X_HIGH && currentY > ZoneConstantsAndyMark.TRENCH_2_DEADZONE_Y_LOW && currentY < ZoneConstantsAndyMark.TRENCH_2_DEADZONE_Y_HIGH) {
            currentZone = ZoneState.TRENCH_DEADZONE;
        }
        else if(currentX > ZoneConstantsAndyMark.TRENCH_34_DEADZONE_X_LOW && currentX < ZoneConstantsAndyMark.TRENCH_34_DEADZONE_X_HIGH && currentY > ZoneConstantsAndyMark.TRENCH_3_DEADZONE_Y_LOW && currentY < ZoneConstantsAndyMark.TRENCH_3_DEADZONE_Y_HIGH) {
            currentZone = ZoneState.TRENCH_DEADZONE;
        }
        else if(currentX > ZoneConstantsAndyMark.TRENCH_34_DEADZONE_X_LOW && currentX < ZoneConstantsAndyMark.TRENCH_34_DEADZONE_X_HIGH && currentY > ZoneConstantsAndyMark.TRENCH_4_DEADZONE_Y_LOW && currentY < ZoneConstantsAndyMark.TRENCH_4_DEADZONE_Y_HIGH) {
            currentZone = ZoneState.TRENCH_DEADZONE;
        }
        else if(currentX > ZoneConstantsAndyMark.MID_ZONE_2_DEADZONE_X_LOW && currentX < ZoneConstantsAndyMark.MID_ZONE_2_DEADZONE_X_HIGH && currentY > ZoneConstantsAndyMark.MID_ZONE_2_DEADZONE_Y_LOW && currentY < ZoneConstantsAndyMark.MID_ZONE_2_DEADZONE_Y_HIGH) {
            currentZone = ZoneState.BACK_DEADZONE;
        }

    }


    public boolean inZone(ZoneState currentZone) {
        return false;
    }

    /**
     * Updates vision measurement standard deviations for fusion/filtering.
     */
    public void setVisionMeasurementStdDevs(double xMeters, double yMeters, double thetaRadians) {
        this.visionStdDevX = xMeters;
        this.visionStdDevY = yMeters;
        this.visionStdDevTheta = thetaRadians;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta));
    }

    public void setMaxDiffForFusion(double translationMeters, double rotationRadians) {
        this.maxTranslationDiff = translationMeters;
        this.maxRotationDiff = rotationRadians;
    }

    @SuppressWarnings("static-access")
    public void setDrivetrainInstance(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // -------------------------
    // Inner classes
    // -------------------------

    /**
     * Wrapper class for vision measurements.
     * Contains pose, confidence, timestamp, and validity flag.
     */
    private record VisionMeasurement(Pose2d pose, double confidence, double timestamp, boolean valid) {
        static VisionMeasurement invalid() {
            return new VisionMeasurement(null, 0.0, 0.0, false);
        }

        boolean isValid() {
            return valid && pose != null;
        }
    }
}