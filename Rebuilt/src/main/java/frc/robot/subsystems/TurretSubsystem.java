package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TurretConstants;

/**
 * Subsystem controlling the turret mechanism on the robot.
 * The turret rotates independently of the robot chassis to aim at targets.
 * 
 * Gear Ratio: Every motor revolution = 1.5 degrees of turret rotation
 * Range: 0 to 270 degrees (0 to 180 motor revolutions)
 */
public class TurretSubsystem extends SubsystemBase {
    
    private static TurretSubsystem INSTANCE;

    /**
     * State machine for turret operation modes
     */
    public enum UsableStates{
        ZEROING,  // Finding the home/zero position
        IDLING,   // Holding current position
        AIMING,   // Actively tracking/aiming at a target
        STORING   // Moving to storage/safe position
    }

    // Current and previous states for state machine logic
    private UsableStates currentState = UsableStates.ZEROING;
    private UsableStates previousState = currentState;

    // TalonFX motor controller for turret rotation
    private TalonFX turretMotor = new TalonFX(TurretConstants.MotorIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    TalonFXConfiguration motorConfig;

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
    private double currentMotorRevs = 0;      // Motor shaft revolutions
    private double currentTurretAngle = 0;    // Turret angle in degrees

    // Target positions for aiming state
    private double targetAimingAngle = 0;
    private double targetAimingMotorRevs = 0;
    
    // Target positions for idling state
    private double targetIdleAngle = 0;
    private double targetIdleMotorRevs = 0;

    // Unclamped target angle (may exceed turret limits)
    private double trueTargetAimingAngle = 0;

    // State tracking flags
    private boolean isCurrentlyZeroing = true;
    private boolean stateNewlyChanged = true;

    /**
     * Constructor - initializes the turret subsystem and configures the motor
     */
    public TurretSubsystem() {
        configureMotors();
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

        // Enable forward software limit at 270 degrees (180 motor revolutions)
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 180;

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

        // Apply configuration to the motor
        turretMotor.getConfigurator().apply(motorConfig);
    }

    /**
     * Periodic update function called every 20ms by the command scheduler
     */
    @Override
    public void periodic() {
        // Update current position from encoder
        currentMotorRevs = turretMotor.getPosition().getValueAsDouble();
        currentTurretAngle = currentMotorRevs * 1.5;

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
            case ZEROING:
                // Apply negative voltage to rotate toward hard stop
                turretMotor.setVoltage(-1);
                
                // Detect when motor stalls at hard stop (high voltage, low velocity)
                if (turretMotor.getMotorVoltage().getValueAsDouble() > 0.8 && turretMotor.getVelocity().getValueAsDouble() < 0.1) {
                    turretMotor.setVoltage(0);
                    turretMotor.setPosition(0);  // Reset encoder to zero
                    isCurrentlyZeroing = false;
                    setState(UsableStates.AIMING);
                }
                break;

            case IDLING:
                // On first entry to idle state, capture current position as target
                if (stateNewlyChanged) {
                    targetIdleAngle = currentTurretAngle;
                    targetIdleMotorRevs = currentMotorRevs;
                }
                // Hold current position using Motion Magic
                turretMotor.setControl(controlRequest.withPosition(targetIdleMotorRevs));
                break;

            case AIMING:
                // Track target position using Motion Magic
                turretMotor.setControl(controlRequest.withPosition(targetAimingMotorRevs));
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

        Logger.recordOutput("Rebuilt/Turret/Currently Zeroing?", isCurrentlyZeroing);

        Logger.recordOutput("Rebuilt/Turret/Motor Revolutions", currentMotorRevs);
        Logger.recordOutput("Rebuilt/Turret/Turrent Angle", currentTurretAngle);

        Logger.recordOutput("Rebuilt/Turret/Target Aiming Angle", targetAimingAngle);
        Logger.recordOutput("Rebuilt/Turret/Target Aiming Motor Revs", targetAimingMotorRevs);
        Logger.recordOutput("Rebuilt/Turret/Target Idle Angle", targetIdleAngle);
        Logger.recordOutput("Rebuilt/Turret/Target Idle Motor Revs", targetIdleMotorRevs);
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
        if (currentState == UsableStates.IDLING) return Math.abs(targetIdleMotorRevs - currentMotorRevs) < tolerance;
        return Math.abs(targetAimingMotorRevs - currentMotorRevs) < tolerance;
    }

    /**
     * Changes the current state of the turret (blocked during zeroing)
     * @param wantedState The desired state to transition to
     */
    public void setState(UsableStates wantedState) {
        if (isCurrentlyZeroing) return;  // Cannot change state while zeroing

        previousState = currentState;
        currentState = wantedState;
        stateNewlyChanged = true;
    }

    /**
     * Sets the aiming target in motor revolutions
     * Clamps value to valid range [0, 180] motor revs
     * @param motorRevs Target position in motor revolutions
     */
    public void setAimingTargetMotorRevs(double motorRevs) {
        trueTargetAimingAngle = motorRevs * 1.5;

        // Clamp to valid range
        motorRevs = MathUtil.clamp(motorRevs, 0, 180);

        targetAimingMotorRevs = motorRevs;
        targetAimingAngle = motorRevs * 1.5;
    }

    /**
     * Sets the aiming target in degrees
     * Clamps value to valid range [0, 180] degrees
     * @param angle Target angle in degrees
     */
    public void setAimingTargetAngle(double angle) {
        trueTargetAimingAngle = angle;

        // Clamp to valid range
        angle = MathUtil.clamp(angle, 0, 180);

        targetAimingAngle = angle;
        targetAimingMotorRevs = angle / 1.5;
    }

    /**
     * @return Current state of the turret
     */
    public UsableStates getCurrentState() {
        return currentState;
    }

    /**
     * @return Current turret angle in degrees
     */
    public double getCurrentAngle() {
        return currentTurretAngle;
    }

    /**
     * @return Current motor position in revolutions
     */
    public double getCurrentMotorRevs() {
        return currentMotorRevs;
    }

    /**
     * @return Target angle based on current state (idle or aiming)
     */
    public double getTargetAngle() {
        if (currentState == UsableStates.IDLING) return targetIdleAngle;
        return targetAimingAngle;
    }

    /**
     * @return Target motor revolutions based on current state
     */
    public double getTargetMotorRevs() {
        if (currentState == UsableStates.IDLING) return targetIdleMotorRevs;
        return targetAimingMotorRevs;
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
}