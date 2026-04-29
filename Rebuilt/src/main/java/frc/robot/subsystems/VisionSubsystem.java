package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants.TurretConstants;
import frc.robot.Constants.RobotConstants.LimelightConstants.BackLeft;
import frc.robot.Constants.RobotConstants.LimelightConstants.BackRight;
import frc.robot.Constants.RobotConstants.LimelightConstants.FrontLeft;
import frc.robot.Constants.RobotConstants.LimelightConstants.FrontRight;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ImportantFieldPoses;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneConstantsWelded;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneTargets;
import frc.robot.util.BezierCurve;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightObject;
import frc.robot.util.LimelightHelpers.IMUData;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.LimelightObject.Location;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * VisionSubsystem using Limelights with:
 *  - Fusion of multiple cameras
 *  - Singleton pattern
 *  - Integration with drivetrain for pose estimation
 *
 * This subsystem reads Limelight data, fuses multiple camera poses,
 * and provides utility functions like distance to target and shooter recommendations.
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------
    // Singleton instance
    // -------------------------
    private static volatile VisionSubsystem instance;

    /**
     * Singleton accessor method.
     * Ensures only one instance exists and is globally accessible.
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

    private volatile Pose2d lastVisionPose = new Pose2d();  // Last pose estimated from fused vision
    private volatile double lastVisionTimestamp = 0.0; // Timestamp of last vision update

    private boolean bumpDetected = false;
    private double bumpMeasurmentCount = 5;
    private boolean odomPoseOutsideField = false;
    private boolean operatorForcedVisionTrust = false;

    // Maximum allowed differences for fusion; if exceeded, fallback to higher confidence camera
    private double maxTranslationDiff = 2.5; // meters
    private double maxRotationDiff = Math.toRadians(45); // radians

    // Validation thresholds for pose filtering
    private static final double MAX_Z_THRESHOLD = 0.25; // Robot must be on ground
    private static final double MAX_PITCH_THRESHOLD = 1000; // Robot must not be tilted forward/back
    private static final double MAX_ROLL_THRESHOLD = 1000; // Robot must not be tilted side-to-side
    private static final double MAX_MT2_YAW_DIFF_DEGREES = 0.5; // Vision yaw must match Pigeon within MAX_YAW_DIFF degrees
    private static final double MAX_MT1_YAW_DIFF_DEGREES = 2.25;

    // Limelight camera objects
    private final List<LimelightObject> LIMELIGHT_LIST = new ArrayList<>();


    // DEBUGGING
    private LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Vision", false);
    // -------------------------
    // Constructor (private for singleton)
    // -------------------------

    private VisionSubsystem() {

        // Configure camera offsets and add to internal list
        configureLimelights();

        // Initialize drivetrain vision standard deviations
        setVisionMeasurementStdDevs(VecBuilder.fill(defaultVisionStdDevX, defaultVisionStdDevY, defaultVisionStdDevTheta));
    }

    /**
     * Initializes LimelightObject instances with physical offsets and names.
     * Ensures that all camera poses are correctly transformed to robot frame.
     */
    private void configureLimelights() {
        // Front Left Limelight
        LimelightObject limelightFL = new LimelightObject(FrontLeft.NAME)
                                        .withLocation(Location.FRONT_LEFT)
                                        .withPose3d(FrontLeft.POSE);
        LIMELIGHT_LIST.add(limelightFL);

        // Front Right Limelight
        LimelightObject limelightFR = new LimelightObject(FrontRight.NAME)
                                        .withLocation(Location.FRONT_RIGHT)
                                        .withPose3d(FrontRight.POSE);
        LIMELIGHT_LIST.add(limelightFR);

        // Back Left Limelight
        LimelightObject limelightBL = new LimelightObject(BackLeft.NAME)
                                        .withLocation(Location.BACK_LEFT)
                                        .withPose3d(BackLeft.POSE);
        LIMELIGHT_LIST.add(limelightBL);

        // Back Right Limelight
        LimelightObject limelightBR = new LimelightObject(BackRight.NAME)
                                        .withLocation(Location.BACK_RIGHT)
                                        .withPose3d(BackRight.POSE);
        LIMELIGHT_LIST.add(limelightBR);
    }

    /**
     * Periodic method called every ~20ms by the scheduler.
     * Updates vision data and logs key metrics.
     */
    @Override
    public void periodic() {
        publishLogs();   // Print/log fusion confidence and pose
    }

    // -------------------------
    // Vision update + fusion
    // -------------------------

    /**
     * Updates pose estimates from all Limelight cameras and fuses them.
     * Sends fused pose to drivetrain for odometry correction.
     */
    public Optional<Pose2d> updateVisionMT2(double robotYaw, double robotYawRate) {

        // Logger.recordOutput("Vision/Odom Pose outside Field", odomPoseOutsideField);
        // Logger.recordOutput("Vision/Bump Detected", bumpDetected);

        List<limelightNetworkTableData> llDataList = new ArrayList<>(LIMELIGHT_LIST.size());
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            String name = limelight.getName();
            LimelightHelpers.SetRobotOrientation_NoFlush(name, robotYaw, robotYawRate, 0, 0, 0, 0);
            boolean hasTarget = LimelightHelpers.getTV(name);

            if (hasTarget) {
                llDataList.add(new limelightNetworkTableData(
                    name,
                    true,
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name),
                    LimelightHelpers.getBotPose_wpiBlue(name),
                    LimelightHelpers.getRawFiducials(name),
                    LimelightHelpers.getIMUData(name)
                ));
            } else {
                llDataList.add(new limelightNetworkTableData(name, false, null, null, null, null));
            }
        }

        // === STEP 2: Process snapshots — no more NT reads below this line ===
        List<VisionMeasurement> measurements = new ArrayList<>(llDataList.size());
        for (limelightNetworkTableData snap : llDataList) {
            if (!snap.hasATarget) continue;
            VisionMeasurement vm = createVisionMeasurement(snap, robotYaw);
            if (vm.isValid()) {
                measurements.add(vm);
                // Logger.recordOutput("Vision/" + snap.name + "/Pose 3D", snap.megaTagEstimate.pose);
            }
        }

        // === STEP 3: Fuse ===
        Optional<Pose2d> fusedPose = fuseVision(measurements);
        fusedPose.ifPresent(pose -> {
            lastVisionPose = pose;
            lastVisionTimestamp = measurements.stream()
                .mapToDouble(VisionMeasurement::timestamp)
                .max().orElse(0.0);
            
            drivetrain.addVisionMeasurement(pose, Utils.currentTimeToFPGATime(lastVisionTimestamp));

        });

        // Store snapshots for logging to reuse — no re-reads needed
        // Logger.recordOutput("Vision/ValidCameraCount", measurements.size());
    }

    public Optional<Pose2d> updateVisionMT1(double robotYaw) {

        for (LimelightObject limelight : LIMELIGHT_LIST) {
            PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());
            if (mt1Estimate != null && mt1Estimate.tagCount >= 1) {
                double sampleRobotYaw = robotYaw;
                double limelightYaw = mt1Estimate.pose.getRotation().getDegrees();
                if (Math.abs(sampleRobotYaw - limelightYaw) < MAX_MT1_YAW_DIFF_DEGREES) {
                    return Optional.of(mt1Estimate.pose);
                }
            }
        }
        return Optional.empty();
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
    private VisionMeasurement createVisionMeasurement(limelightNetworkTableData llData, double pigeonYaw) {
        if (llData == null || llData.megaTag2Estimate == null) return VisionMeasurement.invalid();

        double ambiguity = computeAmbiguity(llData);
        double timestamp = Utils.fpgaToCurrentTime(llData.megaTag2Estimate.timestampSeconds);
        int tagCount = llData.megaTag2Estimate.tagCount;

        if (tagCount == 0 || ambiguity > 0.8) return VisionMeasurement.invalid();
        
        boolean poseValid = true;
        boolean llIMUValid = true;
        
        if (llData.megaTag2Estimate != null) {
            // Extract 3D pose data from botpose array
            // botpose format: [x, y, z, roll, pitch, yaw, ...]
            Pose2d botpose = llData.megaTag2Estimate.pose;
            double[] llPose = {llData.imuData.Roll, llData.imuData.Pitch};
            
            if (botpose != null) {
                double x = botpose.getX();
                double y = botpose.getY();
                double visionYaw = botpose.getRotation().getDegrees();

                double llRoll = llPose[0];
                double llPitch = llPose[1];
                
                // Validation checks
                boolean xValid = (x > 0) && (x < 16.565);
                boolean yValid = (y > 0) && (y < 8.095);

                boolean llRollValid = Math.abs(llRoll) < 20;
                boolean llPitchValid = Math.abs(-28.1 - llPitch) < 20;
                
                
                // Yaw difference check (handle angle wrapping)
                double yawDiff = Math.abs(visionYaw - pigeonYaw);
                // Normalize to [-180, 180]
                while (yawDiff > 180) yawDiff -= 360;
                while (yawDiff < -180) yawDiff += 360;
                boolean yawValid = true;
                if (DriverStation.isEnabled()) {
                    yawValid = Math.abs(yawDiff) <= MAX_MT2_YAW_DIFF_DEGREES;
                }
                
                // Pose is only valid if all checks pass
                poseValid = xValid && yValid && yawValid;
                // llIMUValid = llRollValid && llPitchValid;
                llIMUValid = true;
                
                // Optional: Log which check failed for debugging
                if (!poseValid) {
                    if (!yawValid) Logger.recordOutput("Vision/" + llData.name + "/InvalidYawDiff", yawDiff);
                }
            }
        }

        // Consider measurement valid if at least 1 tag, low ambiguity, AND pose validation passes
        boolean valid = tagCount > 0 && poseValid && llIMUValid && ambiguity < 0.5;

        // Confidence weighted: 70% ambiguity, 30% tag count
        double tagFactor = Math.min(tagCount / 2.0, 1.0);
        double confidence = (1.0 - ambiguity) * 0.7 + tagFactor * 0.3;

        return new VisionMeasurement(llData.megaTag2Estimate.pose, confidence, timestamp, valid);
    }

    /**
     * Computes the best (lowest) ambiguity among all fiducials seen by a Limelight.
     * @Returns 1.0 if no fiducials detected (completely ambiguous).
     */
    private double computeAmbiguity(limelightNetworkTableData llData) {
        LimelightHelpers.RawFiducial[] fiducials = llData.fiducials;
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
        // Filter valid
        int validCount = 0;
        double sumX = 0, sumY = 0;
        VisionMeasurement best = null;

        for (VisionMeasurement m : measurements) {
            if (!m.isValid()) continue;
            sumX += m.pose().getX();
            sumY += m.pose().getY();
            if (best == null || m.confidence() > best.confidence()) best = m;
            validCount++;
        }

        if (validCount == 0) return Optional.empty();
        if (validCount == 1) return Optional.of(best.pose());

        double meanX = sumX / validCount;
        double meanY = sumY / validCount;

        // Single pass for inlier filtering + weighted fusion
        double totalConf = 0, fusedX = 0, fusedY = 0;
        int inlierCount = 0;
        VisionMeasurement highestConfInlier = null;

        for (VisionMeasurement m : measurements) {
            if (!m.isValid()) continue;
            double dx = m.pose().getX() - meanX;
            double dy = m.pose().getY() - meanY;
            double dist = Math.sqrt(dx*dx + dy*dy);
            double rotDiff = Math.abs(m.pose().getRotation().minus(
                new Rotation2d(Math.atan2(meanY, meanX))).getRadians()); // or use first pose rotation

            if (dist < maxTranslationDiff && rotDiff < maxRotationDiff) {
                fusedX += m.pose().getX() * m.confidence();
                fusedY += m.pose().getY() * m.confidence();
                totalConf += m.confidence();
                inlierCount++;
                if (highestConfInlier == null || m.confidence() > highestConfInlier.confidence())
                    highestConfInlier = m;
            }
        }

        if (inlierCount == 0) return Optional.of(best.pose());
        if (inlierCount == 1) return Optional.of(highestConfInlier.pose());

        double rotation = highestConfInlier.pose().getRotation().getRadians();
        return Optional.of(new Pose2d(fusedX / totalConf, fusedY / totalConf, new Rotation2d(rotation)));
    }

    // -------------------------
    // Logging
    // -------------------------
    private void publishLogs() {

        // Overall vision state
        Logger.recordOutput("Vision/Total/Last Vision Pose", lastVisionPose);
        Logger.recordOutput("Vision/Total/Last Vision Timestamp", lastVisionTimestamp);

        if (logDebuggingToggle.get()) {
            
            // Standard deviations
            Logger.recordOutput("Vision/Config/StdDev X", currentVisionStdDevX);
            Logger.recordOutput("Vision/Config/StdDev Y", currentVisionStdDevY);
            Logger.recordOutput("Vision/Config/StdDev Theta", currentVisionStdDevTheta);

            Logger.recordOutput("Vision/Operator Overriden Vision Trust", operatorForcedVisionTrust);
            
            // Fusion thresholds
            Logger.recordOutput("Vision/Config/Max Translation Diff", maxTranslationDiff);
            Logger.recordOutput("Vision/Config/Max Rotation Diff", Math.toDegrees(maxRotationDiff));

            // Per-camera data
            for (LimelightObject limelight : LIMELIGHT_LIST) {
                String name = limelight.getName();
                LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
                
                if (results != null) {
                    Logger.recordOutput("Vision/" + name + "/Valid", results.valid);
                    Logger.recordOutput("Vision/" + name + "/Priority Tag", LimelightHelpers.getFiducialID(name));
                    Logger.recordOutput("Vision/" + name + "/Pipeline Latency", results.latency_pipeline);
                    Logger.recordOutput("Vision/" + name + "/Capture Latency", results.latency_capture);
                    
                    // Get botpose data if available
                    double[] botpose = LimelightHelpers.getBotPose_wpiBlue(name);
                    if (botpose != null && botpose.length >= 6) {
                        Logger.recordOutput("Vision/" + name + "/Botpose/X", botpose[0]);
                        Logger.recordOutput("Vision/" + name + "/Botpose/Y", botpose[1]);
                        Logger.recordOutput("Vision/" + name + "/Botpose/Z", botpose[2]);
                        Logger.recordOutput("Vision/" + name + "/Botpose/Roll", botpose[3]);
                        Logger.recordOutput("Vision/" + name + "/Botpose/Pitch", botpose[4]);
                        Logger.recordOutput("Vision/" + name + "/Botpose/Yaw", botpose[5]);
                    }
                }
            }

            // Fault detection
            Logger.recordOutput("Vision/Faults/Limelight List Empty?", LIMELIGHT_LIST.isEmpty());
            Logger.recordOutput("Vision/Faults/Limelight Count", LIMELIGHT_LIST.size());
            

        }
    }

    // -------------------------
    // Public API
    // -------------------------
    public Pose2d getLastVisionPose() { return lastVisionPose; }

    public void setThrottle(int framesToSkip) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.SetThrottle(limelight.getName(), framesToSkip);
        }
    }

    public void setLLIMUModes(int mode) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.SetIMUMode(limelight.getName(), mode); // Use internal IMU + external IMU
        }
    }

    public void setLLIMUAssistAlpha(double alphaValue) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.SetIMUAssistAlpha(limelight.getName(), alphaValue);
        }
    }

    public void setLLPipeline(int pipeline) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.setPipelineIndex(limelight.getName(), pipeline);
        }
    }

    public void setMaxDiffForFusion(double translationMeters, double rotationRadians) {
        this.maxTranslationDiff = translationMeters;
        this.maxRotationDiff = rotationRadians;
    }

    public Command setOperatorOverrideVisionTrustCommand(boolean value) {
        return new InstantCommand(() -> operatorForcedVisionTrust = value);
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

    private record limelightNetworkTableData(String name, 
                                             boolean hasATarget, 
                                             PoseEstimate megaTag2Estimate,
                                             double[] botPose, 
                                             RawFiducial[] fiducials,
                                             IMUData imuData
                                            ) {}
}