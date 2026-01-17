package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance;

    public enum AngleState {
        ZEROING,
        SHOOTING,
        FERRYING_TOP,
        FERRYING_BOTTOM,
        DEADZONE,
        OVERRIDE,
        STORING
    }

    public enum ShooterState {
        SHOOTING,
        RAMP_UP,
        NOT_SHOOTING,

    }
    private CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
    private VisionSubsystem visionSub = VisionSubsystem.getInstance();

    private AngleState currentAngleState;
    private ShooterState currentShooterState;

    private double targetHoodHeight = 0;
    private double targetShooterVeloc = 0;

    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterLeaderMotor = new TalonFX(ShooterConstants.DeviceIDs.kShooterMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterFollowerMotor = new TalonFX(ShooterConstants.DeviceIDs.kShooterMotorID, RobotConstants.CANIVORE_BUS);

    // Hood Motion Magic parameters
    private LoggedNetworkNumber logHoodMMVeloc = new LoggedNetworkNumber("/Tunable/Shooter/Angle/Velocity", 4);
    private LoggedNetworkNumber logHoodMMAccel = new LoggedNetworkNumber("/Tunable/Shooter/Angle/Acceleration", 4);
    private LoggedNetworkNumber logHoodMMJerk = new LoggedNetworkNumber("/Tunable/Shooter/Angle/Jerk", 400);
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

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();

    private boolean isZeroingDone = false;

    InterpolatingDoubleTreeMap shootingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap shootingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap ferryingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ferryingDistanceToVelocity = new InterpolatingDoubleTreeMap();
=


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
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpoint", targetHoodHeight);
        Logger.recordOutput("Rebuilt/Shooter/Hood/current motor revolutions", angleMotor.getPosition().getValueAsDouble());
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
                angleMotor.setControl(hoodMMRequest.withPosition(null));
                break;
            case FERRYING_BOTTOM:
                angleMotor.setControl(hoodMMRequest.withPosition(null));

                break;
            default:
                break;
        }
    }

    public double getShooterVeloc() {
        switch (coordSub.getCurrentZone()) {
            case SHOOTING_ZONE:
                return shootingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
            case FERRY_ZONE_TOP:
                return ferryingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
                break;
            case FERRY_ZONE_BOTTOM:
                break;
            case TRENCH_DEADZONE:
                break;
            default:
                return 0;
        }
    }

    public double getShootingDistanceToTime(double distance) {
        return shootingDistanceToTime.get(distance);
    }

    public double getFerryingDistanceToTime(double distance) {
        return ferryingDistanceToTime.get(distance);
    }

    public double getShooterAngle() {
        switch (coordSub.getCurrentZone()) {
            case SHOOTING_ZONE:
                shootingHoodAngleFunction();
                break;
            case FERRY_ZONE_TOP:
                break;
            case FERRY_ZONE_BOTTOM:
                break;
            default:
                break;
        }

    }

    public double shootingHoodAngleFunction(double distance) {

        return 0; // Change later when you get the equation
    }

    public double ferryingHoodAngleFunction(double distance) {

        return 0; // Change later when you get the equation
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
