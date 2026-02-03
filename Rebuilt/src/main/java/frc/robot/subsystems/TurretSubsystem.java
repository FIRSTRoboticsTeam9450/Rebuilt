package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.FieldConstants.ImportantFieldPoses;
import frc.robot.Constants.RobotConstants.TurretConstants;

/**
 * Subsystem controlling the turret mechanism with field-relative tracking.
 * 
 * <p>Features:
 * <ul>
 *   <li>Field-relative aiming with velocity feedforward</li>
 *   <li>Automatic compensation for robot rotation</li>
 *   <li>Intelligent angle wrapping for shortest path</li>
 *   <li>Smooth motion profiling</li>
 *   <li>Dynamic target switching (Hub, Ferry, Custom positions)</li>
 *   <li>Alliance-aware targeting for FRC 2026 Reefscape</li>
 * </ul>
 * 
 * <p>Physical Constraints:
 * <ul>
 *   <li>Range: -150° to +150° (300° total)</li>
 *   <li>Gear Ratio: {@link TurretConstants#DEGREE_CHANGE_PER_MOTOR_REVOLUTION} degrees per motor revolution</li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {
    
    // ========================= Hardware =========================
    
    private static volatile TurretSubsystem INSTANCE;
    
    private final TalonFX turretMotor;
    private final TalonFXSimState turretMotorSimState;
    private final CANcoder turretEncoder;
    private final CANcoderSimState turretEncoderSimState;
    private final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private CoordinationSubsystem coordSub;
    
    private CommandXboxController driverController;
    
    // ========================= Configuration =========================
    
    // Physical limits (degrees) - 330° total range
    private static final double MIN_ANGLE_DEG = -(TurretConstants.TOTAL_ROTATION_DEGREES / 2);
    private static final double MAX_ANGLE_DEG = (TurretConstants.TOTAL_ROTATION_DEGREES / 2);
    
    // Conversion constants
    private final double angleOffsetToZero = TurretConstants.TURRET_OFFSET_TO_ZERO;
    private final double encoderToAngleMultiplier = TurretConstants.ABS_ENCDR_POS_TO_ANGLE;
    
    // Tunable parameters
    private final LoggedNetworkNumber logMaxVelocityDegPerSec = 
        new LoggedNetworkNumber("/Tuning/Turret/MaxVelocityDegPerSec", 360); //360
    private final LoggedNetworkNumber logMaxAccelerationDegPerSecSq = 
        new LoggedNetworkNumber("/Tuning/Turret/MaxAccelerationDegPerSecSq", 720); //720
    
    private final LoggedNetworkNumber logKP = new LoggedNetworkNumber("/Tuning/Turret/kP", 80); //200, needs tuning though
    private final LoggedNetworkNumber logKI = new LoggedNetworkNumber("/Tuning/Turret/kI", 0.0);
    private final LoggedNetworkNumber logKD = new LoggedNetworkNumber("/Tuning/Turret/kD", 0.0);
    private final LoggedNetworkNumber logKS = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.4); //5, needs tuning
    private final LoggedNetworkNumber logKV = new LoggedNetworkNumber("/Tuning/Turret/kV",15); //5, needs tuning
    private final LoggedNetworkNumber logKA = new LoggedNetworkNumber("/Tuning/Turret/kA", 0); //50, needs tuning though

    // Motor configuration
    private TalonFXConfiguration motorConfig;
    
    // ========================= Control =========================
    
    /**
     * Turret operating states.
     */
    public enum TurretState {
        /** Do nothing, sit there and look pretty */
        DISABLED,
        /** Goes to store, then once there, goes to aiming */
        INIT,
        /** Holding current position, no active tracking */
        AIMING,
        /** Moving to safe storage position (0°) */
        STORING
    }
    
    /**
     * Pre-defined targeting modes for common game actions.
     */
    public enum TargetMode {
        /** Aiming at alliance Hub for shooting */
        HUB,
        /** Aiming at ferry target for passing */
        FERRY,
        /** Aiming at custom position set via setCustomTarget() */
        CUSTOM
    }
    
    private TurretState currentState = TurretState.INIT;
    private TurretState previousState = currentState;
    
    // Target management
    private TargetMode currentTargetMode = TargetMode.HUB;
    private Translation2d customTargetPosition = null;
    
    // Field-relative targeting
    private Rotation2d fieldRelativeGoalAngle = Rotation2d.fromDegrees(0);
    // private double fieldRelativeGoalVelocityRadPerSec = 0.0;
    
    // Robot-relative targeting (calculated each cycle)
    private double robotRelativeTargetAngleDeg = 0.0;
    private double robotRelativeTargetVelocityDegPerSec = 0.0;
    private double previousRobotRelativeTargetAngleDeg = 0.0;
    
    // State tracking
    private double turretOffset = 0.0;
    
    // Motion profiling
    private SlewRateLimiter velocityLimiter;
    private double currentRateLimit;
    private double profiledSetpointDeg = 0.0;
    
    // Control requests
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private final SimpleMotorFeedforward feedforward;

    //Testing Stuff
    public double testTargetSetpoint = 0;
    
    // ========================= Constructor =========================
    
    /**
     * Creates a new TurretSubsystem.
     * 
     * @param drivetrain Reference to drivetrain for field-relative tracking
     */
    public TurretSubsystem() {
        
        // Initialize hardware
        turretMotor = new TalonFX(
            TurretConstants.DeviceIDs.kMotorID, 
            RobotConstants.CANIVORE_BUS
        );
        turretMotorSimState = turretMotor.getSimState();

        turretEncoder = new CANcoder(
            TurretConstants.DeviceIDs.kAbsEncoderID, 
            RobotConstants.CANIVORE_BUS
        );
        turretEncoderSimState = turretEncoder.getSimState();
        
        // Configure hardware
        configureEncoder();
        configureMotor();
        
        // Initialize feedforward controller
        feedforward = new SimpleMotorFeedforward(
            logKS.get(), 
            logKV.get(), 
            logKA.get()
        );
        
        // Initialize motion profiler
        velocityLimiter = new SlewRateLimiter(
            Math.toRadians(logMaxAccelerationDegPerSecSq.get())
        );
        currentRateLimit = Math.toRadians(logMaxAccelerationDegPerSecSq.get());
    }
    
    // ========================= Configuration =========================
    
    /**
     * Configures the CANcoder absolute encoder.
     */
    private void configureEncoder() {
        //IRL
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.ENCODER_MAGNET_OFFSET;
        
        turretEncoder.getConfigurator().apply(encoderConfig);

        //Sim
        turretEncoderSimState.Orientation = ChassisReference.Clockwise_Positive;
        turretEncoderSimState.SensorOffset = TurretConstants.ENCODER_MAGNET_OFFSET;
    }
    
    /**
     * Configures the TalonFX motor controller with PID and limits.
     */
    private void configureMotor() {
        //IRL
        motorConfig = new TalonFXConfiguration();
        
        // Neutral mode
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // PID gains (Slot 0)
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(logKP.get())
            .withKI(logKI.get())
            .withKD(logKD.get())
            .withKS(logKS.get())
            .withKV(logKV.get())
            .withKA(logKA.get());
        
        motorConfig.Slot0 = slot0;
        
        // Fused CANcoder feedback
        motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_SENSOR_RATIO;
        motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        // Apply configuration
        turretMotor.getConfigurator().apply(motorConfig);

        //SIM
        turretMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        turretMotorSimState.setMotorType(MotorType.KrakenX60);
        turretMotorSimState.addRotorPosition(0);
    }
    
    // ========================= Periodic =========================
    
    @Override
    public void periodic() {
        
        // Update target based on current mode
        updateTargetFromMode();
        setCustomTarget(VisionSubsystem.getInstance().getAdjustedCurrentTarget());

        // Get current robot state
        // Rotation2d robotAngle = drivetrain.getState().Pose.getRotation();
        // ChassisSpeeds robotVelocity = drivetrain.getState().Speeds;
        // double robotOmegaRadPerSec = robotVelocity.omegaRadiansPerSecond;
        
        // Convert field-relative goal to robot-relative
        Rotation2d robotRelativeGoalAngle = fieldRelativeGoalAngle.plus(new Rotation2d(Units.degreesToRadians(angleOffsetToZero - 5)));
        // double robotRelativeGoalVelocityRadPerSec = 
        //     fieldRelativeGoalVelocityRadPerSec - robotOmegaRadPerSec;
        
        // Find best angle considering wrapping and limits
        previousRobotRelativeTargetAngleDeg = robotRelativeTargetAngleDeg;
        robotRelativeTargetAngleDeg = findBestWrappedAngle(robotRelativeGoalAngle.getDegrees());

        // if (robotRelativeTargetAngleDeg < MIN_ANGLE_DEG || robotRelativeTargetAngleDeg > MAX_ANGLE_DEG) {
        //     robotRelativeTargetVelocityDegPerSec = 0.0;
        // } else {
        //     robotRelativeTargetVelocityDegPerSec = (robotRelativeTargetAngleDeg - previousRobotRelativeTargetAngleDeg) / 0.020;
        // }

        robotRelativeTargetVelocityDegPerSec = 0.0;
        
        // robotRelativeTargetVelocityDegPerSec = 0;

        // Driver feedback for out-of-range targets
        updateDriverFeedback();
        
        // Apply state machine
        applyState();
        
        // Publish telemetry
        publishLogs();
    }
    
    /**
     * Updates the field-relative goal based on the current target mode.
     * Called every periodic cycle to recalculate aim point.
     */
    private void updateTargetFromMode() {
        // Translation2d targetPosition = getCurrentTargetPosition();
        Translation2d targetPosition = VisionSubsystem.getInstance().getAdjustedCurrentTarget();
        
        if (targetPosition != null) {
            
            
            // Calculate vector from robot to target
            Translation2d robotToTarget = targetPosition; //targetPosition.minus(robotPose.getTranslation());
            
            // Calculate angle to target
            Rotation2d angleToTarget = new Rotation2d(
                robotToTarget.getX(), 
                robotToTarget.getY()
            );

            
            // Update field-relative goal (target is stationary, so velocity = 0)
            fieldRelativeGoalAngle = angleToTarget;
        }
    }

    public double updateTargetFromMode(Translation2d targetPosition) {
        double targetAngle = 0;

        if (targetPosition != null) {
            Pose2d robotPose = VisionSubsystem.getInstance().getCurrentBotPoseAllianceUnspecific();
            
            // Calculate vector from robot to target
            Translation2d robotToTarget = targetPosition.minus(robotPose.getTranslation());
            
            // Calculate angle to target
            Rotation2d angleToTarget = new Rotation2d(
                robotToTarget.getX(), 
                robotToTarget.getY()
            );
            
            // Update field-relative goal (target is stationary, so velocity = 0)
            targetAngle = angleToTarget.getDegrees();
        }

        return targetAngle;
    }
    
    /**
     * Gets the current target position based on active targeting mode.
     * 
     * @return Target position on field, or null if no valid target
     */
    private Translation2d getCurrentTargetPosition() {
        return switch (currentTargetMode) {
            case HUB -> FieldConstants.isBlueAlliance 
                ?  ImportantFieldPoses.BLUE_HUB_CENTER
                :  ImportantFieldPoses.RED_HUB_CENTER;
            
            case FERRY -> FieldConstants.isBlueAlliance 
                ? drivetrain.getState().Pose.getY() > 4.0345
                    ? ImportantFieldPoses.BLUE_LEFT_FERRY_TARGET
                    : ImportantFieldPoses.BLUE_RIGHT_FERRY_TARGET
                : drivetrain.getState().Pose.getY() > 4.0345
                    ? ImportantFieldPoses.RED_LEFT_FERRY_TARGET
                    : ImportantFieldPoses.RED_RIGHT_FERRY_TARGET;
            
            case CUSTOM -> customTargetPosition;
        };
    }
    
    /**
     * State machine implementation.
     */
    private void applyState() {
        switch (currentState) {
            case DISABLED:
                turretMotor.stopMotor();
                break;

            case INIT:
                if (isAtGoal(20) && ShooterSubsystem.getInstance().getCurrentAngleState() != ShooterSubsystem.AngleState.ZEROING) {
                    currentState = TurretState.AIMING;
                }

                profiledSetpointDeg = -48;

                profiledSetpointDeg = MathUtil.clamp(
                    profiledSetpointDeg, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );
                
                commandTurret(profiledSetpointDeg, 0.0, 0.0);
                break;

            case AIMING:
                // Track target with velocity feedforward
                double maxVelocity = logMaxVelocityDegPerSec.get();
                
                // Apply motion profiling for smooth acceleration
                profiledSetpointDeg = Math.toDegrees(
                    velocityLimiter.calculate(Math.toRadians(robotRelativeTargetAngleDeg))
                );
                
                // Clamp to physical limits
                profiledSetpointDeg = MathUtil.clamp(
                    profiledSetpointDeg, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );
                
                // Clamp velocity for safety
                double clampedVelocity = MathUtil.clamp(
                    robotRelativeTargetVelocityDegPerSec,
                    -maxVelocity,
                    maxVelocity
                );
                
                // Calculate acceleration for feedforward (derivative of velocity setpoint)
                // For now, assume 0 unless tracking accelerating target
                double accelerationDegPerSecSq = 0.0;
                
                commandTurret(profiledSetpointDeg, clampedVelocity, accelerationDegPerSecSq);
                break;
                
            case STORING:
                // Return to zero position with no velocity
                profiledSetpointDeg = -48;

                profiledSetpointDeg = MathUtil.clamp(
                    profiledSetpointDeg, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );
                
                commandTurret(profiledSetpointDeg, 0.0, 0.0);
                break;
        }
    }
    
    /**
     * Commands the turret with position, velocity, and acceleration feedforward.
     * 
     * @param positionDeg Target position in degrees (robot-relative)
     * @param velocityDegPerSec Target velocity in deg/s (robot-relative)
     * @param accelerationDegPerSecSq Target acceleration in deg/s² (robot-relative)
     */
    private void commandTurret(double positionDeg, double velocityDegPerSec, double accelerationDegPerSecSq) {

        positionDeg = MathUtil.clamp(
                    positionDeg, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );

        // Convert degrees to motor rotations
        double positionMotorRevs = positionDeg;
        double velocityMotorRevsPerSec = velocityDegPerSec;
        double accelerationMotorRevsPerSecSq = accelerationDegPerSecSq;
        
        // Convert to encoder units (accounting for offset and encoder ratio)
        double positionEncoderRevs = 
            (positionMotorRevs) / encoderToAngleMultiplier;
        
        // Calculate feedforward voltage
        // FF = kS*sgn(v) + kV*v + kA*a
        double feedforwardVolts = feedforward.calculate(
            velocityMotorRevsPerSec,
            accelerationMotorRevsPerSecSq
        );
        
        // Send position command with feedforward
        turretMotor.setControl(
            positionRequest
                .withPosition(positionEncoderRevs - (turretOffset / encoderToAngleMultiplier))
                .withFeedForward(feedforwardVolts)
        );
    }
    
    // ========================= Helper Methods =========================
    
    /**
     * Finds the best equivalent angle considering wrapping and physical limits.
     * Chooses the angle that requires the least turret rotation.
     * 
     * <p>With 300° range (-150° to +150°), the turret can nearly complete a full rotation,
     * so we check multiple wrapped versions to find the shortest path.
     * 
     * @param targetDegrees Target angle in degrees (robot-relative)
     * @return Best wrapped angle within [-150°, +150°]
     */
    private double findBestWrappedAngle(double targetDegrees) {
        double currentAngle = getCurrentAngleDeg();
        
        double bestAngle = targetDegrees;
        double minDistance = Double.POSITIVE_INFINITY;
        
        // Check wrapped versions: -720°, -360°, 0°, +360°, +720°
        // With 300° range, wrapping becomes more important
        for (int wraps = -2; wraps <= 2; wraps++) {
            double wrappedAngle = targetDegrees + (wraps * 360.0);
            
            // Skip if outside physical limits
            if (wrappedAngle < MIN_ANGLE_DEG || wrappedAngle > MAX_ANGLE_DEG) {
                continue;
            }
            
            // Calculate travel distance for this option
            double distance = Math.abs(currentAngle - wrappedAngle);
            
            // Update if this is the shortest path found so far
            if (distance < minDistance) {
                bestAngle = wrappedAngle;
                minDistance = distance;
            }
        }

        Logger.recordOutput("Rebuilt/Turret/Debugging/Best Angle Unclamped", bestAngle);

        // If no valid angle found (all out of range), clamp to nearest limit
        if (minDistance == Double.POSITIVE_INFINITY) {
            bestAngle = MathUtil.clamp(targetDegrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        }
        
        Logger.recordOutput("Rebuilt/Turret/Debugging/Best Angle Clamped", bestAngle);
        
        return bestAngle;
    }
    
    /**
     * Checks if the current target is outside the turret's physical range.
     * 
     * @return true if target cannot be reached without robot rotation
     */
    private boolean isTargetOutOfRange() {
        return robotRelativeTargetAngleDeg <= MIN_ANGLE_DEG || 
               robotRelativeTargetAngleDeg >= MAX_ANGLE_DEG;
    }
    
    /**
     * Updates tunable parameters if they've changed.
     */
    public void updateTunableParameters() {
        boolean gainsChanged = false;
        boolean profileChanged = false;
        
        // Check if PID/FF gains changed
        if (motorConfig.Slot0.kP != logKP.get() ||
            motorConfig.Slot0.kI != logKI.get() ||
            motorConfig.Slot0.kD != logKD.get() ||
            motorConfig.Slot0.kS != logKS.get() ||
            motorConfig.Slot0.kV != logKV.get() ||
            motorConfig.Slot0.kA != logKA.get()) {
            
            gainsChanged = true;
        }
        
        // Check if profile parameters changed
        double newRateLimit = Math.toRadians(logMaxAccelerationDegPerSecSq.get());
        if (currentRateLimit != newRateLimit) {
            profileChanged = true;
            currentRateLimit = newRateLimit;
        }
        
        // Reconfigure if needed
        if (gainsChanged) {
            configureMotor();

            feedforward.setKa(logKA.get());
            feedforward.setKs(logKS.get());
            feedforward.setKv(logKV.get());
        }
        
        if (profileChanged) {
            velocityLimiter = new SlewRateLimiter(
                Math.toRadians(logMaxAccelerationDegPerSecSq.get())
            );
        }
    }
    
    /**
     * Provides rumble feedback when target is out of range.
     */
    private void updateDriverFeedback() {
        if (driverController != null) {
            if (currentState == TurretState.AIMING && isTargetOutOfRange() && DriverStation.isEnabled() && !DriverStation.isAutonomous()) {
                driverController.setRumble(RumbleType.kBothRumble, 0.05);
            } else {
                driverController.setRumble(RumbleType.kBothRumble, 0.0);
            }
        }
    }
    
    // ========================= Logging =========================
    
    /**
     * Publishes telemetry data to AdvantageKit Logger.
     */
    private void publishLogs() {
        // State information
        Logger.recordOutput("Rebuilt/Turret/CurrentState", currentState);
        Logger.recordOutput("Rebuilt/Turret/PreviousState", previousState);
        Logger.recordOutput("Rebuilt/Turret/TargetMode", currentTargetMode);
        
        // Position tracking
        Logger.recordOutput("Rebuilt/Turret/CurrentAngleDeg", getCurrentAngleDeg());
        Logger.recordOutput("Rebuilt/Turret/TargetAngleDeg", robotRelativeTargetAngleDeg);
        Logger.recordOutput("Rebuilt/Turret/ProfiledSetpointDeg", profiledSetpointDeg);
        Logger.recordOutput("Rebuilt/Turret/FieldGoalAngleDeg", fieldRelativeGoalAngle.getDegrees());
        
        // Velocity tracking
        Logger.recordOutput("Rebuilt/Turret/CurrentVelocityDegPerSec", 
            turretMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/TargetVelocityDegPerSec", 
            robotRelativeTargetVelocityDegPerSec);
        
        // Target information
        Translation2d currentTargetTranslation = getCurrentTargetPosition();
        if (currentTargetTranslation != null) {
            Logger.recordOutput("Rebuilt/Turret/CurrentTargetTranslation", 
                new double[] {currentTargetTranslation.getX(), currentTargetTranslation.getY()});
        }

        Pose2d currentTargetPose = new Pose2d(getCurrentTargetPosition(), new Rotation2d(0));
        Logger.recordOutput("Rebuilt/Turret/CurrentTargetPose", currentTargetPose);
        
        // Hardware data
        Logger.recordOutput("Rebuilt/Turret/MotorRevolutions", 
            turretMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/AbsoluteEncoderPosition", 
            turretEncoder.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/MotorOutputVolts", 
            turretMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/MotorOutputAmps", 
            turretMotor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/MotorVelocity (RPS)", 
            turretMotor.getVelocity().getValueAsDouble());
        
        // Status flags
        Logger.recordOutput("Rebuilt/Turret/AtGoal", isAtGoal());
        Logger.recordOutput("Rebuilt/Turret/TargetOutOfRange", isTargetOutOfRange());

        Logger.recordOutput("Rebuilt/Turret/Debugging/MIN Angle", MIN_ANGLE_DEG);
        Logger.recordOutput("Rebuilt/Turret/Debugging/MAX Angle", MAX_ANGLE_DEG);
    }
    
    // ========================= Public API =========================
    
    /**
     * Sets the target mode to Hub (for shooting).
     * Target will automatically switch based on alliance.
     */
    public void setTargetHub() {
        currentTargetMode = TargetMode.HUB;
    }
    
    /**
     * Sets the target mode to Ferry (for passing).
     * Target will automatically switch based on alliance.
     */
    public void setTargetFerry() {
        currentTargetMode = TargetMode.FERRY;
    }
    
    /**
     * Sets a custom target position for aiming.
     * Useful for dynamic targets or special game actions.
     * 
     * @param targetPosition Field position (Translation2d) to aim at
     */
    public void setCustomTarget(Translation2d targetPosition) {
        currentTargetMode = TargetMode.CUSTOM;
        customTargetPosition = targetPosition;
    }
    
    /**
     * Sets the field-relative target for the turret to track.
     * This is a lower-level method for direct angle control.
     * 
     * @param angle Field-relative angle to point at
     * @param velocityRadPerSec Angular velocity of target in field frame (0 for stationary)
     */
    public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
        this.fieldRelativeGoalAngle = angle;
        // this.fieldRelativeGoalVelocityRadPerSec = velocityRadPerSec;
    }
    
    /**
     * Changes the turret's operating state.
     * 
     * @param wantedState Desired state
     */
    public void setState(TurretState wantedState) {
        previousState = currentState;
        currentState = wantedState;
        
        // Reset motion profile on state change
        velocityLimiter.reset(Math.toRadians(getCurrentAngleDeg()));
    }

    public Command setStateCommand(TurretState state) {
        return new InstantCommand(() -> setState(state));
    }
    
    /**
     * Convenience method to start aiming at Hub.
     * Equivalent to: setState(AIMING) + setTargetHub()
     */
    public void aimAtHub() {
        setTargetHub();
        setState(TurretState.AIMING);
    }
    
    /**
     * Convenience method to start aiming at Ferry target.
     * Equivalent to: setState(AIMING) + setTargetFerry()
     */
    public void aimAtFerry() {
        setTargetFerry();
        setState(TurretState.AIMING);
    }
    
    /**
     * Convenience method to start aiming at custom position.
     * Equivalent to: setState(AIMING) + setCustomTarget(position)
     * 
     * @param targetPosition Custom field position (Translation2d) to aim at
     */
    public void aimAtCustom(Translation2d targetPosition) {
        setCustomTarget(targetPosition);
        setState(TurretState.AIMING);
    }
    
    /**
     * Zeros the turret encoder at the current position.
     * Call this when the turret is at its mechanical zero position.
     */
    public void zero() {
        turretOffset = -turretEncoder.getPosition().getValueAsDouble() * encoderToAngleMultiplier;
    }
    
    /**
     * Resets encoder values to zero.
     */
    public void resetEncoders() {
        turretMotor.setPosition(0);
    }
    
    /**
     * Sets the driver controller for rumble feedback.
     * 
     * @param controller Driver's Xbox controller
     */
    public void setDriverController(CommandXboxController controller) {
        this.driverController = controller;
    }
    
    // ========================= Getters =========================
    
    /**
     * Gets the current turret angle in degrees (robot-relative).
     * 
     * @return Current angle in degrees, range [-150, +150]
     */
    public double getCurrentAngleDeg() {
        return turretEncoder.getPosition().getValueAsDouble() / TurretConstants.SENSOR_TO_MECHANISM_RATIO * encoderToAngleMultiplier + turretOffset;
    }
    
    /**
     * Gets the current turret state.
     * 
     * @return Current operating state
     */
    public TurretState getCurrentState() {
        return currentState;
    }
    
    /**
     * Gets the current target mode.
     * 
     * @return Current targeting mode
     */
    public TargetMode getTargetMode() {
        return currentTargetMode;
    }
    
    /**
     * Gets the field-relative target angle.
     * 
     * @return Target angle in field frame
     */
    public Rotation2d getFieldRelativeTarget() {
        return fieldRelativeGoalAngle;
    }
    
    /**
     * Gets the robot-relative target angle (after wrapping).
     * 
     * @return Target angle in robot frame (degrees)
     */
    public double getRobotRelativeTargetDeg() {
        return robotRelativeTargetAngleDeg;
    }
    
    /**
     * Gets the current target position on the field.
     * 
     * @return Target position, or null if no valid target
     */
    public Translation2d getCurrentTarget() {
        return getCurrentTargetPosition();
    }
    
    /**
     * Checks if turret is within tolerance of target.
     * 
     * @return true if within default tolerance (1.0°)
     */
    public boolean isAtGoal() {
        return isAtGoal(5);
    }
    
    /**
     * Checks if turret is within specified tolerance of target.
     * 
     * @param toleranceDeg Allowable error in degrees
     * @return true if within tolerance
     */
    public boolean isAtGoal(double toleranceDeg) {
        double error = Math.abs(getCurrentAngleDeg() - profiledSetpointDeg);
        return error < toleranceDeg;
    }
    
    /**
     * Gets how far the target is beyond the turret's range.
     * Positive means target is beyond max limit, negative means below min limit.
     * 
     * @return Degrees past available range (0 if within range)
     */
    public double getDegreesPastRange() {
        if (robotRelativeTargetAngleDeg > MAX_ANGLE_DEG) {
            return robotRelativeTargetAngleDeg - MAX_ANGLE_DEG;
        } else if (robotRelativeTargetAngleDeg < MIN_ANGLE_DEG) {
            return robotRelativeTargetAngleDeg - MIN_ANGLE_DEG;
        }
        return 0.0;
    }
    
    // ========================= Singleton =========================
    
    /**
     * Gets the singleton instance of TurretSubsystem.
     * 
     * @param drivetrain Drivetrain reference (required on first call)
     * @return The TurretSubsystem instance
     */
    public static synchronized TurretSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TurretSubsystem();
            INSTANCE.initizalize();
        }
        return INSTANCE;
    }

    public void initizalize() {
        coordSub = CoordinationSubsystem.getInstance();
    }
}