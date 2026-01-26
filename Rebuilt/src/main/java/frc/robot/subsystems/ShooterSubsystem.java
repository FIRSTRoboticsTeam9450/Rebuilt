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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants.HoodConstants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance;

    // States
    public enum AngleState {
        ZEROING,
        AIMING,
        TRENCH,
        DEADZONE,
        OVERRIDE,
        IDLING,
        STORING
    }

    public enum ShooterState {
        SHOOTING,
        RAMP_UP,
        NOT_SHOOTING,

    }

    // Other subsystems
    private CoordinationSubsystem coordSub; // = RobotContainer.coordSub;
    private VisionSubsystem visionSub; // = RobotContainer.visionSub;

    private AngleState currentAngleState = AngleState.STORING;
    private ShooterState currentShooterState = ShooterState.NOT_SHOOTING;

    private double targetHoodHeight = 0;
    private double targetHoodAngleOverride = 0;
    private double targetShooterVeloc = 0;
    private double currentPos = 0;
    private boolean storeCurrentPos = false;

    private double hoodAngleMax = 13.5;
    private double hoodAngleMin = 5;
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 15;

    // Motors
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.CANIVORE_BUS);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();


    // Hood Motion Magic parameters
    private LoggedNetworkNumber logHoodMMVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Velocity", 800);
    private LoggedNetworkNumber logHoodMMAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Acceleration", 800);
    private double hoodMMVelocity = logHoodMMVeloc.get();
    private double hoodMMAcceleration = logHoodMMAccel.get();

    private LoggedNetworkNumber logHoodMMKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", .1);
    private LoggedNetworkNumber logHoodMMKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0.095);
    private LoggedNetworkNumber logHoodMMKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0);
    private LoggedNetworkNumber logHoodMMKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 15);
    private LoggedNetworkNumber logHoodMMKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
    private LoggedNetworkNumber logHoodMMKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
    private LoggedNetworkNumber logHoodMMKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0.3);
    private DynamicMotionMagicVoltage hoodMMRequest;

    // Shooter Outtake Velocity Motion Magic Parameters
    private LoggedNetworkNumber logOuttakeVKS = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kS", 0);
    private LoggedNetworkNumber logOuttakeVKV = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kV", 0.15);
    private LoggedNetworkNumber logOuttakeVKA = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kA", 0);
    private LoggedNetworkNumber logOuttakeVKP = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kP", .5);
    private LoggedNetworkNumber logOuttakeVKFF = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kFF", 0);
    private LoggedNetworkNumber logOuttakeVAccel = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Acceleration", 15);
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

        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration);//.withEnableFOC(RobotConstants.ENABLE_FOC);;
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(outtakeVSlot).withEnableFOC(RobotConstants.ENABLE_FOC);

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
                                                               .withMotionMagicCruiseVelocity(logHoodMMVeloc.get());
        angleMotorConfig.MotionMagic = mmConfigs;

        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 12.856934; // Hood angle limit
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


        Slot0Configs slot0Config = new Slot0Configs().withKS(logOuttakeVKS.get())
                                                    .withKV(logOuttakeVKV.get())
                                                    .withKA(logOuttakeVKA.get())
                                                    .withKP(logOuttakeVKP.get());

        shooterMotorsConfig.Slot0 = slot0Config;

        shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();

        shooterLeftMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterMotorsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterRightMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterRightMotor.setControl(new StrictFollower(shooterLeftMotor.getDeviceID()));
    }

    // public boolean zeroEncoder() {
    //     if(Math.abs(angleMotor.getMotorVoltage().getValueAsDouble()) > 0.6 && Math.abs(angleMotor.getVelocity().getValueAsDouble()) < 0.05) {
    //         angleMotor.setPosition(0);
    //         angleMotor.setVoltage(0);
    //         currentAngleState = AngleState.STORING;
    //         return true;
    //     }
    //     return false;
    // }

    public void zeroEncoder() {
        angleMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // if (!isZeroingDone) {
        //     currentAngleState = AngleState.ZEROING;
        //     isZeroingDone = zeroEncoder();
        // }
        applyState();
        publishLogs();
    }

    public void publishLogs() {
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointMotorRevs", targetHoodHeight);
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointDegrees", targetHoodHeight * degreesPerRev + 15);
        Logger.recordOutput("Rebuilt/Shooter/Hood/Took target", hoodAngleFunction(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget())));
        Logger.recordOutput("Rebuilt/Shooter/Hood/voltage", angleMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Hood/current motor revolutions", angleMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Hood/current degrees", angleMotor.getPosition().getValueAsDouble() * degreesPerRev + degreesToZero);

        Logger.recordOutput("Rebuilt/Shooter/Hood/currentState", currentAngleState);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/currentState", currentShooterState);

        Logger.recordOutput("Rebuilt/Shooter/Outtake/velocitySetpoint", targetShooterVeloc);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/current velocity", shooterLeftMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Shooter/Hood/Override hood setpoint", targetHoodAngleOverride);
        Logger.recordOutput("Rebuilt/Shooter/Hood/Overriding hood", isOverrideAngle);

        Logger.recordOutput("Rebuilt/Shooter/Outtake/TargetOverrideVelocity", logTuningShooterVelocity.get());
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Tuning?", isTuningShooterVelocity);

        Logger.recordOutput("Rebuilt/Shooter/Outtake/Left motor voltage", shooterLeftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Right motor voltage", shooterRightMotor.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Shooter/Outtake/Check why", overrideTargetVeloc != logTuningShooterVelocity.get());

        
        
    }

    public void applyState() {
        switch (currentShooterState) {
            case SHOOTING:
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(getShooterVeloc()));
                break;
            case NOT_SHOOTING:
                shooterLeftMotor.setVoltage(0);
            default:
                break;
        }
        switch (currentAngleState) {
            case AIMING:
                angleMotor.setControl(hoodMMRequest.withPosition(getShooterAngle()));
                break;
            case TRENCH:
                angleMotor.setControl(hoodMMRequest.withPosition(0));
            case IDLING:
                if(!storeCurrentPos) {
                    currentPos = angleMotor.getPosition().getValueAsDouble();//MathUtil.clamp(angleMotor.getPosition().getValueAsDouble(), hoodAngleMin, hoodAngleMax);
                    storeCurrentPos = true;
                }
                angleMotor.setControl(hoodMMRequest.withPosition(5));
            case STORING:
                angleMotor.setControl(hoodMMRequest.withPosition(5));
                
            default:
                break;
        }
    }

    public double getShooterVeloc() {
        if(!isTuningShooterVelocity) {
            // switch (coordSub.getCurrentZone()) {
            //     case SHOOTING_ZONE:
            //         targetShooterVeloc = shootingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
            //         break;
            //     case FERRY_ZONE_TOP:
            //         targetShooterVeloc = ferryingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
            //         break;
                    
            //     case FERRY_ZONE_BOTTOM:
            //         targetShooterVeloc = ferryingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
            //         break;
            //     case TRENCH_DEADZONE:
            //         targetShooterVeloc = shootingDistanceToVelocity.get(visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget()));
            //         break;
            //     default:
            //         break;
            // }
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
        double wantedAngle = (-21.0/(distance + 1.07143)) + 42.0;//Math.atan(51/distance); // Min distance away 4.5
        double wantedRevs = getAsRevs(wantedAngle);
        if(wantedRevs != MathUtil.clamp(wantedRevs, hoodAngleMin, hoodAngleMax)) {
            System.out.println("DEBUGGING: Didn't use calculated angle: " + wantedRevs);
        }
        return MathUtil.clamp(wantedRevs, hoodAngleMin, hoodAngleMax);
    }

    public double getHoodAngleDegrees() {
        return angleMotor.getPosition().getValueAsDouble() * degreesPerRev + 15; // add + 15 if you want true angle
    }

    public double getAsRevs(double degrees) {
        return (degrees-15) / degreesPerRev;
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
        return Math.abs(targetShooterVeloc - shooterLeftMotor.getVelocity().getValueAsDouble()) < tolerance;
    }

    public boolean isShooterReady() {
        return (isAngleWithinTolerance() && isVelocityWithinTolerance());
    }

    public void setOverrideModeAngle(boolean isOverrideAngle) {
        this.isOverrideAngle = isOverrideAngle;
    }

    public void setOverrideModeShooter() {
        this.isTuningShooterVelocity = true;
    }

    public void setOverrideHoodAngle(double targetHoodAngleOverride) {
        this.targetHoodAngleOverride = MathUtil.clamp(getAsRevs(targetHoodAngleOverride), hoodAngleMin, hoodAngleMax);
        
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
                    || (shooterMotorsConfig.MotionMagic.MotionMagicAcceleration != logOuttakeVAccel.get());

        if (updateOuttakeVals) {
            shooterMotorsConfig.Slot0.kS = logOuttakeVKS.get();
            shooterMotorsConfig.Slot0.kV = logOuttakeVKV.get();
            shooterMotorsConfig.Slot0.kA = logOuttakeVKA.get();
            shooterMotorsConfig.Slot0.kP = logOuttakeVKP.get();
            outtakeVKFF = logOuttakeVKFF.get();

            shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();
            
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
                    || (angleMotorConfig.MotionMagic.MotionMagicAcceleration != logHoodMMAccel.get());

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

            hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration);//.withEnableFOC(RobotConstants.ENABLE_FOC);
        }
        
    }
    /**
     * Set shooter state
     * @param wantedShooterState
     * @param wantedAngleState
     */
    public void setState(ShooterState wantedShooterState, AngleState wantedAngleState) {
        storeCurrentPos = false;
        currentShooterState = wantedShooterState;
        currentAngleState = wantedAngleState;
    }

    public Command setStateCommand(ShooterState wantedShooterState, AngleState wantedAngleState) {
        return new InstantCommand(() -> setState(wantedShooterState, wantedAngleState));
    }

    public Command setShooterStateCommand(ShooterState wantedShooterState) {
        return setStateCommand(wantedShooterState, currentAngleState);
    }
    public Command setAngleState(AngleState wantedAngleState) {
        return setStateCommand(currentShooterState, wantedAngleState);
    }
    public static ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
            instance.initizalize();
            System.out.println("New shooter instance: " + instance.toString());
        }
        return instance;
    }
    public void initizalize() {
        visionSub = VisionSubsystem.getInstance();
        coordSub = CoordinationSubsystem.getInstance();
    }
}
