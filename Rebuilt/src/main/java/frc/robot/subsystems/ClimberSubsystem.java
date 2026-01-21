package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {

    private static ClimberSubsystem climberInstance;

    private TalonFX climbMotorFront = new TalonFX(0, RobotConstants.CANIVORE_BUS_NAME); //temporary deviceID value
    // private TalonFX climbMotorBack = new TalonFX(ClimberConstants.DeviceIDs.kClimbMotorBackID, RobotConstants.CANIVORE_BUS_NAME);
    private Servo servoTop;
    private Servo servoBottom;

    // --------- all of these should be in a command ----------
    
    // Placeholder name and used to say how many bars the bot will climb up
    //private LoggedNetworkNumber barsToClimb = new LoggedNetworkNumber("/Tuning/Climb/barsToClimb", 1);
     
    // Each one says the distance the motor should move to the next climb 
    // private LoggedNetworkNumber L3Climb = new LoggedNetworkNumber("/Tuning/Climb/L3Climb", 0.0);
    // private LoggedNetworkNumber L2Climb = new LoggedNetworkNumber("/Tuning/Climb/L2Climb", 0.0);
    // private LoggedNetworkNumber L1Climb = new LoggedNetworkNumber("/Tuning/Climb/L1Climb", 0.0);

    private LoggedNetworkNumber[] climbDistances = 
    {  
        new LoggedNetworkNumber("/Tuning/Climb/L3Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L2Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L1Climb", 0.0)
    };

    private int climbCounter = 0;

    private LoggedNetworkNumber logMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/MM/Velocity", 0.1);
    private LoggedNetworkNumber logMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/MM/Acceleration", 1);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Climb/MM/Jerk", 1000);
    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Climb/MM/kS", 0.24);
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Climb/MM/kV", 0.33);
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Climb/MM/kA", 0.1);
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Climb/MM/kP", 0.5);
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Climb/MM/kI", 0.0);
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Climb/MM/kD", 0.0);

    private TalonFXConfiguration climbConfig;
    boolean atTarget;

    private DynamicMotionMagicVoltage controlRequest = 
                        new DynamicMotionMagicVoltage(0, 
                            logMMVelocity.get(), 
                            logMMAcceleration.get())
                            .withJerk(logMMJerk.get())
                            .withSlot(0);
    // MotionMagic type may change

    public enum ClimberStates {
        STANDBY,
        EXTENDCLIMB,
        REELING,
        RETRACTCLIMB    
    }

    private ClimberStates currentState = ClimberStates.STANDBY;

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        climbConfig = new TalonFXConfiguration();

        climbConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logMMKS.get())
                                        .withKV(logMMKV.get())
                                        .withKA(logMMKA.get())
                                        .withKP(logMMKP.get())
                                        .withKI(logMMKI.get())
                                        .withKD(logMMKD.get());

        climbConfig.Slot0 = slot0Configs;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        climbConfig.MotionMagic = mmConfigs;

        
        climbMotorFront.getConfigurator().apply(climbConfig);
        // climbMotorBack.getConfigurator().apply(climbConfig);
        servoTop = new Servo(0); //temporary
        servoBottom = new Servo(0); //temporary

        // climbMotorBack.setControl(new Follower(climbMotorFront.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /* ----------- Updaters ----------- */

     public void setTarget(double setpoint) {
         climbMotorFront.setControl(controlRequest.withPosition(setpoint));
    }

    public void resetEncoders() {
        climbMotorFront.setPosition(0);
        // climbMotorBack.setPosition(0);
    }

    public void resetServos() {
        servoBottom.setAngle(0);
        servoTop.setAngle(0);
    }

    private void publishClimbLogs() {
        Logger.recordOutput("Rebuilt/Climb/MotorPosition",getMotorPosition());
        Logger.recordOutput("Rebuilt/Climb/TopServoAngle",getTopServoAngle());
        Logger.recordOutput("Rebuilt/Climb/BottomServoAngle",getBottomServoAngle());
    }

    public void periodic() {
        applyStates();
        publishClimbLogs();
    }

    public boolean atTarget() {
        atTarget = (Math.abs(climbDistances[climbCounter].get()) - getMotorPosition() < 0.05 && climbMotorFront.getVelocity().getValueAsDouble() < 0.1);
        return atTarget;
    }

    private void applyStates() {
        switch (currentState) {
            case STANDBY:
                break;
            case EXTENDCLIMB:
                // Move the climb a specific distance based on the array
                setTarget(climbDistances[climbCounter].get());

                if(atTarget() == true) {
                    servoTop.set(90);

                    currentState = ClimberStates.RETRACTCLIMB;
                }
                
            case RETRACTCLIMB:
                // retract the climb carry the bot up the bar
                setTarget(climbDistances[climbCounter].get());
                /*
                if (Math.abs(getMotorPosition())  < 0.05) {
                    climbMotorFront.set(0);

                    // possible location for adding bottom servo but it's not added yet so waitting

                    // loop through states based on bars to move up
                    currentClimb += 1;
                    currentState = ClimberStates.EXTENDCLIMB;
                    if (currentClimb >= barsToClimb.get()) {
                        currentState = ClimberStates.STANDBY;
                    }
                }
                */
            default:
                break;
        }
    }

    public double getTopServoAngle() {
        return servoTop.getAngle();
    }

    public double getBottomServoAngle() {
        return servoBottom.getAngle();
    }

    public double getMotorPosition() {
        return climbMotorFront.getPosition().getValueAsDouble();
    }

    public static ClimberSubsystem getInstance() {
        if (climberInstance == null) {
            climberInstance = new ClimberSubsystem();
        }
        System.out.println("acquired" + climberInstance); //temporary

        return climberInstance;
    }
    
}
