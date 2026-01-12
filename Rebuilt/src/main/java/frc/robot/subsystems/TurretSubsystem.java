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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TurretConstants;


/**
 * Notes:
 * Every motor Rev = 1.5 degrees of turret rotation
 */
public class TurretSubsystem extends SubsystemBase {
    
    private static TurretSubsystem INSTANCE;

    public enum UsableStates{
        ZEROING,
        IDLING,
        AIMING,
        STORING
    }

    private UsableStates currentState = UsableStates.ZEROING;
    private UsableStates previousState = currentState;

    private TalonFX turretMotor = new TalonFX(TurretConstants.MotorIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    TalonFXConfiguration motorConfig;

    private LoggedNetworkNumber logMMVelocity = new LoggedNetworkNumber("/Tuning/Turret/Velocity", 0.1);
    private LoggedNetworkNumber logMMAcceleration = new LoggedNetworkNumber("/Tuning/Turret/Velocity", 1);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Turret/Velocity", 1000);
    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.24);
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Turret/kV", 0.33);
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.1);
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.5);
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.0);
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.0);


    
    private DynamicMotionMagicVoltage controlRequest = new DynamicMotionMagicVoltage(0, logMMVelocity.get(), logMMAcceleration.get()).withJerk(logMMJerk.get())
                                                                                                                                              .withSlot(0); 

    private double currentMotorRevs = 0;
    private double currentTurretAngle = 0;

    private double targetAimingAngle = 0;
    private double targetAimingMotorRevs = 0;
    private double targetIdleAngle = 0;
    private double targetIdleMotorRevs = 0;

    private boolean isCurrentlyZeroing = true;
    private boolean stateNewlyChanged = true;

    public TurretSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Get through CAD, this is a place holder.

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 270; // Should be changed to work around gear reductions, in other words, goes from revolutions of motor, to degrees of turret.

        Slot0Configs slot0Configs = new Slot0Configs().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                                      .withKS(logMMKS.get())
                                                      .withKV(logMMKV.get())
                                                      .withKA(logMMKA.get())
                                                      .withKP(logMMKP.get())
                                                      .withKI(logMMKI.get())
                                                      .withKD(logMMKD.get());

        motorConfig.Slot0 = slot0Configs;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        motorConfig.MotionMagic = mmConfigs;

        turretMotor.getConfigurator().apply(motorConfig);
    }


    @Override
    public void periodic() {
        currentMotorRevs = turretMotor.getPosition().getValueAsDouble();
        currentTurretAngle = currentMotorRevs * 1.5;

        applyStates();
        publishLogs();
    }

    private void applyStates() {
        switch (currentState) {
            case ZEROING:
                turretMotor.setVoltage(-1);
                if (turretMotor.getMotorVoltage().getValueAsDouble() > 0.8 && turretMotor.getVelocity().getValueAsDouble() < 0.1) {
                    turretMotor.setVoltage(0);
                    turretMotor.setPosition(0);
                    isCurrentlyZeroing = false;
                    setState(UsableStates.AIMING);
                }
                break;

            case IDLING:
                if (stateNewlyChanged) {
                    targetIdleAngle = currentTurretAngle;
                    targetIdleMotorRevs = currentMotorRevs;
                }
                turretMotor.setControl(controlRequest.withPosition(targetIdleMotorRevs));
                break;

            case AIMING:
                turretMotor.setControl(controlRequest.withPosition(targetAimingMotorRevs));
                break;

            case STORING:
                turretMotor.setControl(controlRequest.withPosition(0));
                break;
        }
    }

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

    public boolean turretWithinTolerance() {
        return turretWithTolerance(0.5);
    }

    public boolean turretWithTolerance(double tolerance) {
        if (currentState == UsableStates.IDLING) return Math.abs(targetIdleMotorRevs - currentMotorRevs) < tolerance;
        return Math.abs(targetAimingMotorRevs - currentMotorRevs) < tolerance;
    }

    public void setState(UsableStates wantedState) {
        if (isCurrentlyZeroing) return;

        previousState = currentState;
        currentState = wantedState;
        stateNewlyChanged = true;
    }

    public void setAimingTargetMotorRevs(double motorRevs) {
        if (motorRevs < 0) motorRevs = 0;
        if (motorRevs > 180) motorRevs = 180;

        targetAimingMotorRevs = motorRevs;
        targetAimingAngle = motorRevs * 1.5;
    }

    public void setAimingTargetAngle(double angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;

        targetAimingAngle = angle;
        targetAimingMotorRevs = angle / 1.5;
    }

    public UsableStates getCurrentState() {
        return currentState;
    }

    public double getCurrentAngle() {
        return currentTurretAngle;
    }

    public double getCurrentMotorRevs() {
        return currentMotorRevs;
    }

    public double getTargetAngle() {
        if (currentState == UsableStates.IDLING) return targetIdleAngle;
        return targetAimingAngle;
    }

    public double getTargetMotorRevs() {
        if (currentState == UsableStates.IDLING) return targetIdleMotorRevs;
        return targetAimingMotorRevs;
    }

    public static TurretSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new TurretSubsystem();
        return INSTANCE;
    }

    /*  =================================
     *  |           Updaters            |
     *  =================================
     */

    /**
     * Checks the values of both motion magic and slot 0 configs, if any value is new, then update them.
     */
    public void updateMotionMagicConstants() {
        if (motorConfig.MotionMagic.MotionMagicAcceleration != logMMAcceleration.get()
            || motorConfig.MotionMagic.MotionMagicCruiseVelocity != logMMVelocity.get()
            || motorConfig.MotionMagic.MotionMagicJerk != logMMJerk.get()) {

            controlRequest = new DynamicMotionMagicVoltage(0, logMMVelocity.get(), logMMAcceleration.get()).withJerk(logMMJerk.get()).withSlot(0);
            configureMotors();
        }

        if (motorConfig.Slot0.kS != logMMKS.get()
            || motorConfig.Slot0.kV != logMMKV.get()
            || motorConfig.Slot0.kA != logMMKA.get()
            || motorConfig.Slot0.kP != logMMKP.get()
            || motorConfig.Slot0.kI != logMMKI.get()
            || motorConfig.Slot0.kD != logMMKD.get()) {

            motorConfig.Slot0.kV = logMMKV.get();
            motorConfig.Slot0.kA = logMMKA.get();
            motorConfig.Slot0.kP = logMMKP.get();
            motorConfig.Slot0.kI = logMMKI.get();
            motorConfig.Slot0.kD = logMMKD.get();

            configureMotors();
        }
        
    }
}
