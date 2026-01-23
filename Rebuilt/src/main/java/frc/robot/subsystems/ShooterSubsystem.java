package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants.HoodConstants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance;

    // States
    public enum AngleState {
        ZEROING,
        SHOOTING,
        TRENCH,
        DEADZONE,
        OVERRIDE,
        STORING
    }

    public enum ShooterState {
        SHOOTING,
        RAMP_UP,
        NOT_SHOOTING,

    }

    // Other subsystems
    private CoordinationSubsystem coordSub = RobotContainer.coordSub;
    private VisionSubsystem visionSub = VisionSubsystem.getInstance();

    private AngleState currentAngleState = AngleState.STORING;
    private ShooterState currentShooterState = ShooterState.NOT_SHOOTING;

    private double targetHoodHeight = 0;
    private double targetHoodAngleOverride = 0;
    private double targetShooterVeloc = 0;

    private double hoodAngleMax = 0;
    private double hoodAngleMin = 0;
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 15;

    // Motors
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterLeaderMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterFollowerMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.CANIVORE_BUS);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();


    // Hood Motion Magic parameters
    private LoggedNetworkNumber logHoodMMVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Velocity", 4);
    private LoggedNetworkNumber logHoodMMAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Acceleration", 4);
    private LoggedNetworkNumber logHoodMMJerk = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Jerk", 400);
    private double hoodMMVelocity = logHoodMMVeloc.get();
    private double hoodMMAcceleration = logHoodMMAccel.get();
    private double hoodMMJerk = logHoodMMJerk.get();

    private LoggedNetworkNumber logHoodMMKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", 0);
    private LoggedNetworkNumber logHoodMMKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0.33);
    private LoggedNetworkNumber logHoodMMKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0.05);
    private LoggedNetworkNumber logHoodMMKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 0.01);
    private LoggedNetworkNumber logHoodMMKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
    private LoggedNetworkNumber logHoodMMKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
    private LoggedNetworkNumber logHoodMMKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0.05);
    private DynamicMotionMagicVoltage hoodMMRequest;
    
    // Shooter Outtake Velocity Motion Magic Parameters
    private LoggedNetworkNumber logOuttakeVKS = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kS", 0.1);
    private LoggedNetworkNumber logOuttakeVKV = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kV", 0.1);
    private LoggedNetworkNumber logOuttakeVKA = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kA", 0.01);
    private LoggedNetworkNumber logOuttakeVKP = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kP", 0.6);
    private LoggedNetworkNumber logOuttakeVKFF = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kFF", 1);
    private LoggedNetworkNumber logOuttakeVAccel = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Acceleration", 100);
    private LoggedNetworkNumber logOuttakeVJerk = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Jerk", 4000);
    private double outtakeVKFF = logOuttakeVKFF.get();
    private final int outtakeVSlot = 0;
    private final MotionMagicVelocityVoltage outtakeVRequest;

    // Tuning velocity table
    private LoggedNetworkNumber logTuningShooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity", 40);
    private LoggedNetworkBoolean logIsTuningShooterVelocity = new LoggedNetworkBoolean("/Tuning/Shooter/Outtake/TuningShooterVelocity?", false);
    
    // Flags
    private boolean isTuningShooterVelocity = logIsTuningShooterVelocity.get();
    private boolean isOverrideAngle = false;
    private boolean isZeroingDone = false;

    // Data tables
    InterpolatingDoubleTreeMap shootingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap shootingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap ferryingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ferryingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        configureAngleMotors();
        configureShooterMotors();

        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withJerk(hoodMMJerk);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(outtakeVSlot);

    }

    public void configureAngleMotors() {
        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;

        Slot0Configs slot0Config = new Slot0Configs().withKS(logHoodMMKS.get())
                                                    .withKV(logHoodMMKV.get())
                                                    .withKA(logHoodMMKA.get())
                                                    .withKP(logHoodMMKP.get())
                                                    .withKI(logHoodMMKI.get())
                                                    .withKD(logHoodMMKD.get())
                                                    .withKG(logHoodMMKG.get());
        angleMotorConfig.Slot0 = slot0Config;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logHoodMMAccel.get())
                                                               .withMotionMagicCruiseVelocity(logHoodMMVeloc.get())
                                                               .withMotionMagicJerk(logHoodMMJerk.get());
        angleMotorConfig.MotionMagic = mmConfigs;

        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.530273; // Hood angle limit
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        angleMotor.getConfigurator().apply(angleMotorConfig);
    }

    private void configureShooterMotors() {
        shooterMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorsConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterMotorsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorsConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLowerTime = 3;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLimit = 40;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

        shooterMotorsConfig.Feedback.SensorToMechanismRatio = 9/8; //1.125

        Slot0Configs slot0Config = new Slot0Configs().withKS(logOuttakeVKS.get())
                                                    .withKV(logOuttakeVKV.get())
                                                    .withKA(logOuttakeVKA.get())
                                                    .withKP(logOuttakeVKP.get());

        shooterMotorsConfig.Slot0 = slot0Config;

        shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();
        shooterMotorsConfig.MotionMagic.MotionMagicJerk = logOuttakeVJerk.get();

        shooterLeaderMotor.getConfigurator().apply(shooterMotorsConfig);
        shooterFollowerMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterFollowerMotor.setControl(new Follower(shooterLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean zeroEncoder() {
        if(Math.abs(angleMotor.getMotorVoltage().getValueAsDouble()) > 0.6 && Math.abs(angleMotor.getVelocity().getValueAsDouble()) < 0.05) {
            angleMotor.setPosition(0);
            angleMotor.setVoltage(0);
            currentAngleState = AngleState.STORING;
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        if (!isZeroingDone) {
            currentAngleState = AngleState.ZEROING;
            isZeroingDone = zeroEncoder();
        }
        applyState();
        publishLogs();
    }

    public void publishLogs() {
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointMotorRevs", targetHoodHeight);
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointDegrees", targetHoodHeight * degreesPerRev);
        Logger.recordOutput("Rebuilt/Shooter/Hood/current motor revolutions", angleMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Hood/current degrees", angleMotor.getPosition().getValueAsDouble() * degreesPerRev + degreesToZero);

        Logger.recordOutput("Rebuilt/Shooter/Hood/currentState", currentAngleState);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/velocitySetpoint", targetShooterVeloc);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/currentState", currentShooterState);
        
    }

    public void applyState() {
        switch (currentShooterState) {
            case SHOOTING:
                shooterLeaderMotor.setControl(outtakeVRequest.withVelocity(getShooterVeloc()));
                break;
            default:
                break;
        }
        switch (currentAngleState) {
            case SHOOTING:
                angleMotor.setControl(hoodMMRequest.withPosition(getShooterAngle()));
                break;
            case TRENCH:
                angleMotor.setControl(hoodMMRequest.withPosition(0));
            default:
                break;
        }
    }

    public double getShooterVeloc() {
        if(!isTuningShooterVelocity) {
            switch (coordSub.getCurrentZone()) {
                case SHOOTING_ZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break;
                case FERRY_ZONE_TOP:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break;
                    
                case FERRY_ZONE_BOTTOM:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break;
                case TRENCH_DEADZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break;
                default:
                    break;
            }
        }
        else {
            targetShooterVeloc = logTuningShooterVelocity.get();
        }
        return targetShooterVeloc;
    }

    public double getShooterAngle() {
        if(!isOverrideAngle) {
            switch (coordSub.getCurrentZone()) {
                case SHOOTING_ZONE:
                    targetHoodHeight = hoodAngleFunction(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break;
                    
                case FERRY_ZONE_TOP:
                    targetHoodHeight = hoodAngleFunction(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break; 
                    
                case FERRY_ZONE_BOTTOM:
                    targetHoodHeight = hoodAngleFunction(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                    break;
                case TRENCH_DEADZONE:
                    targetHoodHeight = 0;
                default:
                    return 0;
            }
        }
        else {
            targetHoodHeight = targetHoodAngleOverride;
        }
        return targetHoodHeight;

    }

    public double getShootingDistanceToTime(double distance) {
        return shootingDistanceToTime.get(distance);
    }

    public double getFerryingDistanceToTime(double distance) {
        return ferryingDistanceToTime.get(distance);
    }

    public double hoodAngleFunction(double distance) {
        double wantedAngle = (-21/(distance + 0.7)) + 14.5;//Math.atan(51/distance); // Min distance away 4.5
        if(wantedAngle != MathUtil.clamp(wantedAngle, hoodAngleMin, hoodAngleMax)) {
            System.out.println("DEBUGGING: Didn't use calculated angle: " + wantedAngle);
        }
        return MathUtil.clamp(wantedAngle, hoodAngleMin, hoodAngleMax);
    }

    public double getHoodAngle() {
        return angleMotor.getPosition().getValueAsDouble() * degreesPerRev; // add + 15 if you want true angle
    }

    public boolean isAngleWithinTolerance() {
        return isAngleWithinTolerance(0.1);
    }

    public boolean isAngleWithinTolerance(double tolerance) {
        return Math.abs(targetHoodHeight - angleMotor.getPosition().getValueAsDouble()) < tolerance;
    }

    public boolean isVelocityWithinTolerance() {
        return isVelocityWithinTolerance(0.5);
    }

    public boolean isVelocityWithinTolerance(double tolerance) {
        return Math.abs(targetShooterVeloc - shooterLeaderMotor.getVelocity().getValueAsDouble()) < tolerance;
    }

    public boolean isShooterReady() {
        return (isAngleWithinTolerance() && isVelocityWithinTolerance());
    }
    public void setOverrideMode(boolean isOverrideAngle) {
        this.isOverrideAngle = isOverrideAngle;
    }

    public void setOverrideHoodAngle(double targetHoodAngleOverride) {
        this.targetHoodAngleOverride = MathUtil.clamp((targetHoodAngleOverride / degreesPerRev), hoodAngleMin, hoodAngleMax);
        
    }

    public void updateShooterConstants() {
        boolean updateOuttakeVals = false;
        boolean updateAngleVals = false;
        boolean updateSetpoints = false;

        // double lVVSVal = logVelocitySetpoint.get(); //Uncomment for velocity tuning
        //double lVoltageSetpoint = logVoltageSetpoint.get(); Uncomment for voltage tuning
        
        // double lDistanceAway = logDistanceAway.get();
        // updateSetpoints = (distanceAway != lDistanceAway)
        //                 // || (velocitySetpoint != lVVSVal)  // Uncomment for velocity tuning
        //                 //|| (voltageSetpoint != lVoltageSetpoint) Uncomment for voltage tuning
        //                 ;
        // if(updateSetpoints) {
        // distanceAway = lDistanceAway;
        // changeOnce = true;
        //voltageSetpoint = lVoltageSetpoint;
        // velocitySetpoint = lVVSVal;  // Uncomment for velocity tuning

        //}
        updateSetpoints = (isTuningShooterVelocity != logIsTuningShooterVelocity.get());
        if(updateSetpoints) {
            isTuningShooterVelocity = logIsTuningShooterVelocity.get();
        }

        updateOuttakeVals = (shooterMotorsConfig.Slot0.kS != logOuttakeVKS.get())
                    || (shooterMotorsConfig.Slot0.kV != logOuttakeVKV.get())
                    || (shooterMotorsConfig.Slot0.kA != logOuttakeVKA.get())
                    || (shooterMotorsConfig.Slot0.kP != logOuttakeVKP.get())
                    || (outtakeVKFF != logOuttakeVKFF.get())
                    || (shooterMotorsConfig.MotionMagic.MotionMagicAcceleration != logOuttakeVAccel.get())
                    || (shooterMotorsConfig.MotionMagic.MotionMagicJerk != logOuttakeVJerk.get());

        if (updateOuttakeVals) {
            shooterMotorsConfig.Slot0.kS = logOuttakeVKS.get();
            shooterMotorsConfig.Slot0.kV = logOuttakeVKV.get();
            shooterMotorsConfig.Slot0.kA = logOuttakeVKA.get();
            shooterMotorsConfig.Slot0.kP = logOuttakeVKP.get();
            outtakeVKFF = logOuttakeVKFF.get();

            shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();
            shooterMotorsConfig.MotionMagic.MotionMagicJerk = logOuttakeVJerk.get();
            
            configureShooterMotors();
        }


        updateAngleVals = (angleMotorConfig.Slot0.kS != logHoodMMKS.get())
                    || (angleMotorConfig.Slot0.kV != logHoodMMKV.get())
                    || (angleMotorConfig.Slot0.kA != logHoodMMKA.get())
                    || (angleMotorConfig.Slot0.kP != logHoodMMKP.get())
                    || (angleMotorConfig.Slot0.kI != logHoodMMKI.get())
                    || (angleMotorConfig.Slot0.kD != logHoodMMKD.get())
                    || (angleMotorConfig.Slot0.kG != logHoodMMKG.get())
                    || (angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logHoodMMVeloc.get())
                    || (angleMotorConfig.MotionMagic.MotionMagicAcceleration != logHoodMMAccel.get())
                    || (angleMotorConfig.MotionMagic.MotionMagicJerk != logHoodMMJerk.get());

        if (updateAngleVals) {
            angleMotorConfig.Slot0.kS = logHoodMMKS.get();
            angleMotorConfig.Slot0.kV = logHoodMMKV.get();
            angleMotorConfig.Slot0.kA = logHoodMMKA.get();
            angleMotorConfig.Slot0.kP = logHoodMMKP.get();
            angleMotorConfig.Slot0.kI = logHoodMMKI.get();
            angleMotorConfig.Slot0.kD = logHoodMMKD.get();
            angleMotorConfig.Slot0.kG = logHoodMMKG.get();

            configureAngleMotors();

            hoodMMVelocity = logHoodMMVeloc.get();
            hoodMMAcceleration = logHoodMMAccel.get();
            hoodMMJerk = logHoodMMJerk.get();

            hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withJerk(hoodMMJerk);
        }
        
    }
    /**
     * Set shooter state
     * @param wantedShooterState
     * @param wantedAngleState
     */
    public void setState(ShooterState wantedShooterState, AngleState wantedAngleState) {
        currentShooterState = wantedShooterState;
        currentAngleState = wantedAngleState;
    }

    public static ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }
}
