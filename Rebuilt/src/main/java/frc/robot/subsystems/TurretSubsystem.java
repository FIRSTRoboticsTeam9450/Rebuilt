package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TurretConstants;

/**
 * Subsystem controlling the turret mechanism with the right operator joystick.
 * 
 * <p>Features:
 * <ul>
 *   <li>Intelligent angle wrapping for shortest path</li>
 *   <li>Smooth joystick control</li>
 * </ul>
 * 
 * <p>Physical Constraints:
 * <ul>
 *   <li>Range: -30° to +30° (60° total)</li>
 *   <li>Gear Ratio: {@link TurretConstants#DEGREE_CHANGE_PER_MOTOR_REVOLUTION} degrees per motor revolution</li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {
    
    // ========================= Hardware =========================
    
    private static volatile TurretSubsystem INSTANCE;
    
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;
    private final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    
    // ========================= Configuration =========================
    
    // Physical limits (degrees) - degree° total range depends on the constant
    private static final double MIN_ANGLE_DEG = -(TurretConstants.TOTAL_ROTATION_DEGREES / 2);
    private static final double MAX_ANGLE_DEG = (TurretConstants.TOTAL_ROTATION_DEGREES / 2);
    
    // Conversion constants
    private final double encoderToAngleMultiplier = TurretConstants.ABS_ENCDR_POS_TO_ANGLE;
    
    private final double allowedEncoderDiff = 0.05;
    
    // Tunable parameters
    
    private final LoggedNetworkNumber logKP = new LoggedNetworkNumber("/Tuning/Turret/kP", 50); // 62.5
    private final LoggedNetworkNumber logKI = new LoggedNetworkNumber("/Tuning/Turret/kI", 0.0);
    private final LoggedNetworkNumber logKD = new LoggedNetworkNumber("/Tuning/Turret/kD", 0.0);
    private final LoggedNetworkNumber logKS = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.21);
    private final LoggedNetworkNumber logKV = new LoggedNetworkNumber("/Tuning/Turret/kV",0.0115);
    private final LoggedNetworkNumber logKA = new LoggedNetworkNumber("/Tuning/Turret/kA", 0);

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Turret", false);

    // Cached values
    private double cachedAngleDeg = 0.0;
    private double cachedMotorVelocity = 0.0;

    // Motor configuration
    private TalonFXConfiguration motorConfig;

    // Controller
    private final CommandXboxController OPERATOR = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
    
    // ========================= Control =========================
    
    /**
     * Turret operating states.
     */
    public enum TurretState {
        /** Holding current position, no active tracking */
        AIMING,
        /** Moving to safe storage position (0°) */
        STORING,

        CLIMB
    }
    
    private TurretState currentState = TurretState.AIMING;
    private TurretState previousState = currentState;
    
    // Robot-relative targeting (calculated each cycle)
    private double robotRelativeTargetAngleDeg = 0.0;
    private double robotRelativeTargetVelocityDegPerSec = 0.0;

    private Rotation2d previousTurretAngleWRTField = new Rotation2d();

    private double previousVelocityTimestamp = 0.0;
    
    // State tracking
    private double turretOffset = 0.0;
    
    // Motion profiling
    private double profiledSetpointDeg = 0.0;
    
    // Control requests
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(false);
    private final SimpleMotorFeedforward feedforward;
    
    // ========================= Constructor =========================
    
    /**
     * Creates a new TurretSubsystem.
     */
    public TurretSubsystem() {

        // Initialize hardware
        turretMotor = new TalonFX(
            TurretConstants.DeviceIDs.kMotorID, 
            RobotConstants.MAIN_SYSTEMS_CANBUS
        );

        turretEncoder = new CANcoder(
            TurretConstants.DeviceIDs.kAbsEncoderID, 
            RobotConstants.MAIN_SYSTEMS_CANBUS
        );
        
        // Configure hardware
        configureEncoder();
        configureMotor();
        
        // Initialize feedforward controller
        feedforward = new SimpleMotorFeedforward(
            logKS.get(), 
            logKV.get(), 
            logKA.get()
        );

        drivetrain.getPigeon2().getAngularVelocityZDevice().setUpdateFrequency(50);
    }
    
    // ========================= Configuration =========================
    
    /**
     * Configures the Encoder <br><br/>
     * Runs upon instance construction <br><br/>
     * Configures:
     *  <ul>
     *  <li> Rotation Direction
     *  <li> Encoder Magnet Offset
     *  <ul/>
     */
    private void configureEncoder() {
        //IRL
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.ENCODER_MAGNET_OFFSET;
        
        turretEncoder.getConfigurator().apply(encoderConfig);
    }
    
    /**
     * Configures the motor <br><br/>
     * Runs upon instance construction, and constant change (tuning mode) <br><br/>
     * Configures:
     *  <ul>
     *  <li> Neutral Mode Value
     *  <li> Rotation Direction
     *  <li> Software Limits
     *  <li> Stator & Supply Current Limits
     *  <li> Peak Motor Voltage
     *  <li> Slot 0 Constants
     *  <li> Remote Sensor Type & Ratios
     *  <li> Peak Tourqe current
     *  <ul/>
     */
    private void configureMotor() {
        //IRL
        motorConfig = new TalonFXConfiguration();
        
        // Neutral mode
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Calculate motor revolutions for your angle limits
        double minMotorRevs = MIN_ANGLE_DEG / TurretConstants.ABS_ENCDR_POS_TO_ANGLE;
        double maxMotorRevs = MAX_ANGLE_DEG / TurretConstants.ABS_ENCDR_POS_TO_ANGLE;

        // Now account for the rotor-to-sensor ratio
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxMotorRevs;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minMotorRevs;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 15;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        motorConfig.Voltage.PeakForwardVoltage = 7;
        motorConfig.Voltage.PeakReverseVoltage = -7;
        
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
    }
    
    // ========================= Periodic =========================
    
    @Override
    public void periodic() {
        cachedAngleDeg = turretEncoder.getAbsolutePosition().getValueAsDouble() / TurretConstants.SENSOR_TO_MECHANISM_RATIO * encoderToAngleMultiplier + turretOffset;

        if (Math.abs(turretEncoder.getAbsolutePosition().getValueAsDouble() - turretEncoder.getPosition().getValueAsDouble()) > allowedEncoderDiff) {
            resetEncodersRelativePosition();
        }
    
        // Set angle from controller joystick
        double x = OPERATOR.getLeftX(); double y = OPERATOR.getLeftY();
        if (Math.abs(x) > 0.5 || Math.abs(y) > 0.5) {
            double angleRadians = Math.atan2(y, x) + (Math.PI / 2);
            double angleDegrees = Math.toDegrees(angleRadians);
            if (angleDegrees > 180) angleDegrees -= 360;

            if (angleDegrees >= -150 && angleDegrees <= 150)
                robotRelativeTargetAngleDeg = MathUtil.clamp(angleDegrees, -30, 30);
        }

        robotRelativeTargetVelocityDegPerSec = calculateVelocityUpdated();
        
        // Apply state machine
        applyState();
        
        // Publish telemetry
        publishLogs();
    }

    /**
     * Calculates the target Velocity in Degrees per Second.
     * <p>Safety Precautions:
     * <ul>
     *   <li>If at deadzone, do not use any velocity</li>
     *   <li>If unclamped velocity will lead to going outside of deadzone, limit velocity to what will not go outside of deadzone</li>
     * </ul>
     * @return the target velocity
     */
    public double calculateVelocityUpdated() {
        if ((robotRelativeTargetAngleDeg <= MIN_ANGLE_DEG || robotRelativeTargetAngleDeg >= MAX_ANGLE_DEG) || (cachedAngleDeg <= MIN_ANGLE_DEG || cachedAngleDeg >= MAX_ANGLE_DEG)) return 0.0;

        double turretVeloictyWRTRobot = 0.0;
        double robotYawVelocWRTField = drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble(); //degrees/second

        Rotation2d robotYaw = drivetrain.getState().Pose.getRotation(); //rotation
        Rotation2d turretAngleWRTField = robotYaw.plus(new Rotation2d(Units.degreesToRadians(robotRelativeTargetAngleDeg))); //radians

        double timestamp = Timer.getFPGATimestamp(); //seconds

        Rotation2d deltaTurretAngleWRTField = turretAngleWRTField.minus(previousTurretAngleWRTField);

        // Calculate desired velocity (derivative of target position)
        double turretVelocityWRTField = (deltaTurretAngleWRTField.getDegrees()) / (timestamp - previousVelocityTimestamp); //degrees/second

        turretVeloictyWRTRobot = turretVelocityWRTField - robotYawVelocWRTField; //degrees/second

        previousTurretAngleWRTField = turretAngleWRTField;
        previousVelocityTimestamp = timestamp;

        return turretVeloictyWRTRobot;
    }
    
    /**
     * State machine implementation.
     */
    private void applyState() {
        switch (currentState) {
            case AIMING:
                // Track target with velocity feedforward
                profiledSetpointDeg = MathUtil.clamp(
                    robotRelativeTargetAngleDeg, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );
                
                commandTurret(profiledSetpointDeg, robotRelativeTargetVelocityDegPerSec, 0.0);
                break;
                
            case STORING:
                // Return to zero position with no velocity
                profiledSetpointDeg = MathUtil.clamp(
                    0, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );

                commandTurret(profiledSetpointDeg, 0.0, 0.0);
                break;
            case CLIMB:
                profiledSetpointDeg = MathUtil.clamp(
                    90, 
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
     * Updates tunable parameters if they've changed.
     */
    public void updateTunableParameters() {
        boolean gainsChanged = false;
        
        // Check if PID/FF gains changed
        if (motorConfig.Slot0.kP != logKP.get() ||
            motorConfig.Slot0.kI != logKI.get() ||
            motorConfig.Slot0.kD != logKD.get() ||
            motorConfig.Slot0.kS != logKS.get() ||
            motorConfig.Slot0.kV != logKV.get() ||
            motorConfig.Slot0.kA != logKA.get()) {
            
            gainsChanged = true;
        }
        
        // Reconfigure if needed
        if (gainsChanged) {
            configureMotor();

            feedforward.setKa(logKA.get());
            feedforward.setKs(logKS.get());
            feedforward.setKv(logKV.get());
        }
    }
    
    // ========================= Logging =========================
    
    /**
     * Publishes telemetry data to AdvantageKit Logger.
     */
    private void publishLogs() {
        // State information
        Logger.recordOutput("Rebuilt/Turret/State/CurrentState", currentState);
        
        // Position tracking
        Logger.recordOutput("Rebuilt/Turret/Pos/CurrentAngleDeg", cachedAngleDeg);
        Logger.recordOutput("Rebuilt/Turret/Target/ProfiledSetpointDeg", profiledSetpointDeg);

        Logger.recordOutput("Rebuilt/Turret/Device/Motor Supply Amps", turretMotor.getSupplyCurrent().getValueAsDouble());
        
        // Velocity tracking
        Logger.recordOutput("Rebuilt/Turret/Target/TargetVelocityDegPerSec", 
            robotRelativeTargetVelocityDegPerSec);

        if (logDebuggingToggle.get()) {
            Logger.recordOutput("Rebuilt/Turret/State/PreviousState", previousState);

            Logger.recordOutput("Rebuilt/Turret/Target/TargetAngleDeg", robotRelativeTargetAngleDeg);
            
            Logger.recordOutput("Rebuilt/Turret/Device/MotorRevolutions", 
                turretMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/AbsoluteEncoderPosition", 
                turretEncoder.getAbsolutePosition().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/Encoder Relative Position", 
                turretEncoder.getPosition().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/MotorOutputVolts", 
                turretMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/MotorOutputAmps", 
                turretMotor.getTorqueCurrent().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/MotorVelocity (RPS)", 
                cachedMotorVelocity);
        }
    }
    
    // ========================= Public API =========================
    
    /**
     * Changes the turret's operating state.
     * 
     * @param wantedState Desired state
     */
    public void setState(TurretState wantedState) {
        previousState = currentState;
        currentState = wantedState;
    }

    public Command setStateCommand(TurretState state) {
        return new InstantCommand(() -> setState(state));
    }
    
    /**
     * Zeros the turret encoder at the current position.
     * Call this when the turret is at its mechanical zero position.
     */
    public void zero() {
        turretOffset = -turretEncoder.getAbsolutePosition().getValueAsDouble() * encoderToAngleMultiplier;
    }
    
    /**
     * Resets the Turret's motor internal encoder values to zero.
     */
    public void resetEncoders() {
        turretMotor.setPosition(0);
    }

    /**
     * Syncs the Turret motor's position with the encoders
     */
    public void syncMotorPosToEncoder() {
        turretMotor.setPosition(cachedAngleDeg / TurretConstants.DEGREE_CHANGE_PER_MOTOR_REVOLUTION);
    }

    public void resetEncodersRelativePosition() {
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    // ========================= Getters =========================
    
    /**
     * Gets the current turret angle in degrees (robot-relative).
     * 
     * @return Current angle in degrees, range [-135, +135]
     */
    public double getCurrentAngleDeg() {
        return cachedAngleDeg;
    }
    
    /**
     * Gets the current turret state.
     * 
     * @return Current operating state
     */
    public TurretState getCurrentState() {
        return currentState;
    }
    
    // ========================= Singleton =========================
    
    /**
     * Gets the singleton instance of TurretSubsystem.
     * 
     * @return The TurretSubsystem instance
     */
    public static synchronized TurretSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TurretSubsystem();
            INSTANCE.initialize();
        }
        return INSTANCE;
    }

    public void initialize() {
        //Logger.recordOutput("Rebuilt/Turret/Subsystem Initizalized", true);
    }

    public TalonFX getMotor() {
        return turretMotor;
    }
}