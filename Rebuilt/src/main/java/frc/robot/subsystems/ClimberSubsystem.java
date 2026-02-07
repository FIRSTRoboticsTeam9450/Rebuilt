package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

    private static volatile ClimberSubsystem climberInstance;

    private TalonFX climbMotorFront = new TalonFX(0, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value

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

    private int climbCounter;

    Timer delayStopwatch = new Timer();

    private DynamicMotionMagicVoltage controlRequest = 
                        new DynamicMotionMagicVoltage(0, 
                            logMMVelocity.get(), 
                            logMMAcceleration.get())
                            .withJerk(logMMJerk.get())
                            .withSlot(0)
                            .withEnableFOC(true);
    // MotionMagic type may change

    public enum ElevatorStates {
        STOWING,
        STANDBY,
        EXTENDING,
        LOCKING, //this will be removed later on
        RETRACTING
    }

    public enum TapeClimbStates {
        STOWING,
        STANDBY,
        EXTENDING,
        LOCKING, //this will be removed later on
        RETRACTING
    }

    private ElevatorStates currentElevatorState = ElevatorStates.STANDBY;
    private TapeClimbStates currentTapeClimbState = TapeClimbStates.STANDBY;

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        configureMotors();
    }

    public void configureMotors() {
        //MOTOR I
        climbConfig = new TalonFXConfiguration();

        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logMMKS.get())
                                        .withKV(logMMKV.get())
                                        .withKA(logMMKA.get())
                                        .withKP(logMMKP.get())
                                        .withKI(logMMKI.get())
                                        .withKD(logMMKD.get());
        
        Slot1Configs slot1Configs = new Slot1Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logMMKS.get())
                                        .withKV(logMMKV.get())
                                        .withKA(logMMKA.get())
                                        .withKP(logMMKP.get())
                                        .withKI(logMMKI.get())
                                        .withKD(logMMKD.get())
                                        .withKG(0);

        climbConfig.Slot0 = slot0Configs;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        climbConfig.MotionMagic = mmConfigs;

        
        climbMotorFront.getConfigurator().apply(climbConfig);

        //MOTOR II
    }

    /* ----------- Updaters ----------- */

    public void setNumberofClimbs(int numClimbs) {
        this.climbCounter = numClimbs;
    }

    public void setTarget(double setpoint) {
         climbMotorFront.setControl(controlRequest.withPosition(setpoint) //in rotations
                                                  .withVelocity(null)); //in rotations per second
    }

    public void resetEncoders() {
        climbMotorFront.setPosition(0);
    }

    private void publishClimbLogs() {
        Logger.recordOutput("Rebuilt/Climb/MotorPosition",getMotorPosition());
    }

    public void periodic() {
        applyStates();
        publishClimbLogs();
    }

    public boolean atTarget() {
        atTarget = (Math.abs(climbDistances[climbCounter].get()) - getMotorPosition() < 0.05 && climbMotorFront.getVelocity().getValueAsDouble() < 0.1);
        return atTarget;
    }

    public boolean endClimbSequence() {
        return climbCounter == 0;
    }

    public void resetStopwatch() {
        delayStopwatch.stop();
        delayStopwatch.reset();
    }

    private void applyStates() {
        //consider merging these two switches
        //stuff to consider- the motion magic configs

        switch (currentElevatorState) {
            case STANDBY:
                setTarget(Math.round(Math.abs(climbMotorFront.getPosition().getValueAsDouble())));
                //502 bad code
                break;
            case EXTENDING:
                setTarget(7);
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.LOCKING;
                }
                break;
            case LOCKING:
                setTarget(5); //temporary value
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.RETRACTING;
                }
                break;
            case RETRACTING:
                setTarget(0);
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.STANDBY;
                }
                break;
            case STOWING:
                setTarget(0);
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.STANDBY;
                } 
                break;
        }
        switch(currentTapeClimbState) {
            case EXTENDING:
                setTarget(7);
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.LOCKING;
                }
                break;
            case RETRACTING:
                setTarget(0);
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.STANDBY;
                }
                break;
            case LOCKING:
                setTarget(5);
                if(atTarget()) {
                    currentElevatorState = ElevatorStates.RETRACTING;
                }
                break;
            case STANDBY:
                setTarget(Math.round(Math.abs(climbMotorFront.getPosition().getValueAsDouble())));
                //make sure to set what happens during state standby
                break;
            case STOWING:
                setTarget(0);
                break;
        }
    }

    public double getMotorPosition() {
        return climbMotorFront.getPosition().getValueAsDouble();
    }

    public static synchronized ClimberSubsystem getInstance() {
        if (climberInstance == null) {
            climberInstance = new ClimberSubsystem();
        }
        System.out.println("acquired" + climberInstance); //temporary

        return climberInstance;
    }
    
}
