package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TurretConstants;

/**
 * Subsystem controlling the turret mechanism on the robot.
 * The turret rotates independently of the robot chassis to aim at targets.
 * 
 * Gear Ratio: Every motor revolution = 1.5 degrees of turret rotation
 * Range: 0 to 270 degrees (-90 to 90 motor revolutions)
 */
public class TurretSubsystem extends SubsystemBase {
    
    private static TurretSubsystem INSTANCE;

    private CommandXboxController driverController;

    /**
     * State machine for turret operation modes
     */
    public enum TurretStates{
        IDLING,   // Holding current position
        AIMING,   // Actively tracking/aiming at a target
        STORING   // Moving to storage/safe position
    }

    // Current and previous states for state machine logic
    private TurretStates currentState = TurretStates.STORING;
    private TurretStates previousState = currentState;

    // TalonFX motor controller for turret rotation
    private TalonFX turretMotor = new TalonFX(TurretConstants.DeviceIDs.kMotorID, RobotConstants.CANIVORE_BUS);
    private CANcoder turretEncoder = new CANcoder(TurretConstants.DeviceIDs.kAbsEncoderID, RobotConstants.CANIVORE_BUS);

    private double angleOffsetToZero = TurretConstants.TURRET_OFFSET_TO_ZERO;
    private double encdrToAngleMulti = TurretConstants.ABS_ENCDR_POS_TO_ANGLE;
    private double degreePerMotorRev = TurretConstants.DEGREE_CHANGE_PER_MOTOR_REVOLUTION;

    private TalonFXConfiguration motorConfig;

    // NetworkTables tunable parameters for motion magic and PID control
    private LoggedNetworkNumber logMMVelocity = new LoggedNetworkNumber("/Tuning/Turret/Velocity", 0.1);
    private LoggedNetworkNumber logMMAcceleration = new LoggedNetworkNumber("/Tuning/Turret/Acceleration", 1);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Turret/Jerk", 1000);
    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.24);  // Static friction feedforward
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Turret/kV", 0.33);  // Velocity feedforward
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Turret/kA", 0.1);   // Acceleration feedforward
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Turret/kP", 0.5);   // Proportional gain
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Turret/kI", 0.0);   // Integral gain
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Turret/kD", 0.0);   // Derivative gain

    // Motion Magic control request for smooth, profiled motion
    private DynamicMotionMagicVoltage controlRequest = new DynamicMotionMagicVoltage(0, logMMVelocity.get(), logMMAcceleration.get()).withJerk(logMMJerk.get())
                                                                                                                                              .withSlot(0); 

    // Current position tracking
    private double currentTurretAngle = 0;    // Turret angle in degrees

    private Rotation2d currentRobotRotation;

    // Target positions for aiming state
    private double targetAimingAngle = 0;
    
    // Target positions for idling state
    private double targetIdleAngle = 0;

    /**Unclamped target angle (may exceed turret limits) */
    private double trueTargetAimingAngle = 0;

    private double currentTurretRotationWRTFieldOrigin;
    private double trueTurretAngleWRTFieldOrigin;

    // State tracking flags
    private boolean stateNewlyChanged = true;

    /**
     * Constructor - initializes the turret subsystem and configures the motor
     */
    public TurretSubsystem() {
        configureEncoders();
        configureMotors();
    }

    private void configureEncoders() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        turretEncoder.getConfigurator().apply(encoderConfig);
    }

    /**
     * Configures the TalonFX motor with Motion Magic, PID, and safety settings
     */
    private void configureMotors() {
        motorConfig = new TalonFXConfiguration();

        // Set neutral mode (brake or coast)
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        
        // Set motor direction (TODO: verify with CAD)
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure PID and feedforward gains (Slot 0)
        Slot0Configs slot0Configs = new Slot0Configs().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                                      .withKS(logMMKS.get())
                                                      .withKV(logMMKV.get())
                                                      .withKA(logMMKA.get())
                                                      .withKP(logMMKP.get())
                                                      .withKI(logMMKI.get())
                                                      .withKD(logMMKD.get());

        motorConfig.Slot0 = slot0Configs;

        // Configure Motion Magic parameters for smooth acceleration profiles
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        motorConfig.MotionMagic = mmConfigs;

        motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_SENSOR_RATIO;
        motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

        // Apply configuration to the motor
        turretMotor.getConfigurator().apply(motorConfig);
    }

    /**
     * Periodic update function called every 20ms by the command scheduler
     */
    @Override
    public void periodic() {
        // Update current position from encoder
        currentTurretAngle = turretEncoder.getPosition().getValueAsDouble() * encdrToAngleMulti;

        //TODO: Do better (Get difference of robot degree to turret degree)
        trueTurretAngleWRTFieldOrigin = (currentRobotRotation.getDegrees() - trueTargetAimingAngle) % 360;
        currentTurretRotationWRTFieldOrigin = (currentRobotRotation.getDegrees() - currentTurretAngle) % 360;
        
        // if (driverController != null) {
        //     if (currentState == TurretStates.AIMING && targetAngleOutsideOfRange()) {
        //         driverController.setRumble(RumbleType.kBothRumble, 0.1);
        //     } else {
        //         driverController.setRumble(RumbleType.kBothRumble, 0);
        //     }
        // }

        // Execute current state logic
        applyStates();
        
        // Log telemetry data
        publishLogs();
    }

    /**
     * State machine implementation - executes logic for current state
     */
    private void applyStates() {
        switch (currentState) {
            case IDLING:
                // On first entry to idle state, capture current position as target
                if (stateNewlyChanged) {
                    stateNewlyChanged = false;
                    targetIdleAngle = currentTurretAngle;
                }
                // Hold current position using Motion Magic
                turretMotor.setControl(controlRequest.withPosition(((targetIdleAngle / degreePerMotorRev) + (angleOffsetToZero / degreePerMotorRev)) / encdrToAngleMulti));
                break;

            case AIMING:
                // Track target position using Motion Magic
                turretMotor.setControl(controlRequest.withPosition(((targetAimingAngle / degreePerMotorRev) + (angleOffsetToZero / degreePerMotorRev)) / encdrToAngleMulti));
                break;

            case STORING:
                // Return to zero/storage position
                turretMotor.setControl(controlRequest.withPosition(0));
                break;
        }
    }

    /**
     * Publishes telemetry data to AdvantageKit Logger
     */
    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Turret/Current State", currentState);
        Logger.recordOutput("Rebuilt/Turret/Previous State", previousState);

        Logger.recordOutput("Rebuilt/Turret/Motor Revolutions", turretMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/Turrent Angle", currentTurretAngle);
        Logger.recordOutput("Rebuilt/Turret/Absolute Position", turretEncoder.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Turret/Encoder to Angle Test", turretEncoder.getPosition().getValueAsDouble() * encdrToAngleMulti);

        Logger.recordOutput("Rebuilt/Turret/Target Aiming Angle", targetAimingAngle);
        Logger.recordOutput("Rebuilt/Turret/Target Aiming Motor Revs", targetAimingAngle / degreePerMotorRev);
        Logger.recordOutput("Rebuilt/Turret/Target Idle Angle", targetIdleAngle);
        Logger.recordOutput("Rebuilt/Turret/Target Idle Motor Revs", targetIdleAngle / degreePerMotorRev);
    }

    /**
     * Checks if turret is within default tolerance (0.5 motor revs) of target
     * @return true if within tolerance
     */
    public boolean turretWithinTolerance() {
        return turretWithTolerance(0.5);
    }

    /**
     * Checks if turret is within specified tolerance of target position
     * @param tolerance Maximum error in motor revolutions
     * @return true if within tolerance
     */
    public boolean turretWithTolerance(double tolerance) {
        if (currentState == TurretStates.IDLING) return Math.abs(targetIdleAngle - currentTurretAngle) < tolerance;
        return Math.abs(targetAimingAngle - currentTurretAngle) < tolerance;
    }

    public boolean targetAngleOutsideOfRange() {
        return trueTargetAimingAngle < 0 || trueTargetAimingAngle > 270;
    }

    public boolean targetAngleOutsideOfRange(double angle) {
        return angle < 0 || angle > 270;
    }

    /**
     * Changes the current state of the turret (blocked during zeroing)
     * @param wantedState The desired state to transition to
     */
    public void setState(TurretStates wantedState) {
        previousState = currentState;
        currentState = wantedState;
        stateNewlyChanged = true;
    }


    /**
     * Sets the aiming target in degrees
     * Clamps value to valid range [-90, 90] degrees
     * @param angle Target angle in degrees
     */
    public void setAimingTargetAngle(double angle) {
        trueTargetAimingAngle = angle;

        // Clamp to valid range
        angle = MathUtil.clamp(angle, -135, 135);

        targetAimingAngle = angle;
    }

    public void setDriverController(CommandXboxController controller) {
        driverController = controller;
    }

    /**
     * @return Current state of the turret
     */
    public TurretStates getCurrentState() {
        return currentState;
    }

    /**
     * @return Current turret angle in degrees
     */
    public double getCurrentAngle() {
        return currentTurretAngle;
    }


    /**
     * @return Target angle based on current state (idle or aiming)
     */
    public double getTargetAngle() {
        if (currentState == TurretStates.IDLING) return targetIdleAngle;
        return targetAimingAngle;
    }

    /**
     * @return Unclamped target angle (may exceed turret limits)
     */
    public double getTrueTargetAngle() {
        return trueTargetAimingAngle;
    }

    /**
     * Calculates how many degrees the target is beyond the turret's range
     * Useful for driver feedback or chassis rotation decisions
     * @return Difference between true target and current angle
     */
    public double getDegreesPastAvaliableTurretRange() {
        return trueTargetAimingAngle - targetAimingAngle;
    }

    /**
     * Singleton pattern - returns the single instance of TurretSubsystem
     * @return The TurretSubsystem instance
     */
    public static TurretSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new TurretSubsystem();
        return INSTANCE;
    }

    /**
     * Checks for changes in Motion Magic and PID parameters from NetworkTables
     * If any tuning values have changed, reconfigures the motor
     * This allows for live tuning without redeploying code
     */
    public void updateMotionMagicConstants() {
        // Check if Motion Magic parameters changed
        if (motorConfig.MotionMagic.MotionMagicAcceleration != logMMAcceleration.get()
            || motorConfig.MotionMagic.MotionMagicCruiseVelocity != logMMVelocity.get()
            || motorConfig.MotionMagic.MotionMagicJerk != logMMJerk.get()) {

            // Recreate control request with new parameters
            controlRequest = new DynamicMotionMagicVoltage(0, logMMVelocity.get(), logMMAcceleration.get()).withJerk(logMMJerk.get()).withSlot(0);
            configureMotors();
        }

        // Check if PID/feedforward gains changed
        if (motorConfig.Slot0.kS != logMMKS.get()
            || motorConfig.Slot0.kV != logMMKV.get()
            || motorConfig.Slot0.kA != logMMKA.get()
            || motorConfig.Slot0.kP != logMMKP.get()
            || motorConfig.Slot0.kI != logMMKI.get()
            || motorConfig.Slot0.kD != logMMKD.get()) {

            // Update gain values
            motorConfig.Slot0.kV = logMMKV.get();
            motorConfig.Slot0.kA = logMMKA.get();
            motorConfig.Slot0.kP = logMMKP.get();
            motorConfig.Slot0.kI = logMMKI.get();
            motorConfig.Slot0.kD = logMMKD.get();

            configureMotors();
        }
    }

    public void updateRobotRotation(Rotation2d robotRotation) {
        currentRobotRotation = robotRotation;
    }
}