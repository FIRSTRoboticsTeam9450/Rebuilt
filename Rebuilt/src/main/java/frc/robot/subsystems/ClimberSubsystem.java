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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

    private static volatile ClimberSubsystem climberInstance;

    private TalonFX tapeClimbMotor = new TalonFX(0, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value
    private TalonFX elevatorClimbMotor = new TalonFX(0, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value

    // --------- all of these should be in a command ----------

    private LoggedNetworkNumber[] climbDistances = 
    {  
        new LoggedNetworkNumber("/Tuning/Climb/L3Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L2Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L1Climb", 0.0)
    };

    private LoggedNetworkNumber logMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/MM/Velocity", 0.1);
    private LoggedNetworkNumber logMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/MM/Acceleration", 1);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Climb/MM/Jerk", 1000);
    
    // for motor 1/ tape motor
    private LoggedNetworkNumber logTapeMMKS = new LoggedNetworkNumber("/Tuning/Climb/Tape/MM/kS", 0.24);
    private LoggedNetworkNumber logTapeMMKV = new LoggedNetworkNumber("/Tuning/Climb/Tape/MM/kV", 0.33);
    private LoggedNetworkNumber logTapeMMKA = new LoggedNetworkNumber("/Tuning/Climb/Tape/MM/kA", 0.1);
    private LoggedNetworkNumber logTapeMMKP = new LoggedNetworkNumber("/Tuning/Climb/Tape/MM/kP", 0.5);
    private LoggedNetworkNumber logTapeMMKI = new LoggedNetworkNumber("/Tuning/Climb/Tape/MM/kI", 0.0);
    private LoggedNetworkNumber logTapeMMKD = new LoggedNetworkNumber("/Tuning/Climb/Tape/MM/kD", 0.0);
    // private LoggedNetworkNumber logKG = new LoggedNetworkNumber("/Tuning/Climb/kG"); not being used

    // for motor 2/ elevator motor
    private LoggedNetworkNumber logElevatorMMKS = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kS", 0.24);
    private LoggedNetworkNumber logElevatorMMKV = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kV", 0.33);
    private LoggedNetworkNumber logElevatorMMKA = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kA", 0.1);
    private LoggedNetworkNumber logElevatorMMKP = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kP", 0.5);
    private LoggedNetworkNumber logElevatorMMKI = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kI", 0.0);
    private LoggedNetworkNumber logElevatorMMKD = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kD", 0.0);

    private TalonFXConfiguration climbConfig1;
    private TalonFXConfiguration climbConfig2;
    boolean atTarget;

    private LoggedNetworkNumber barsToClimb = new LoggedNetworkNumber("/Tuning/Climb/barsToClimb", 1);

    private int climbCounter = 0;

    // Timer delayStopwatch = new Timer();

    private DynamicMotionMagicVoltage controlRequest = 
                        new DynamicMotionMagicVoltage(0, 
                            logMMVelocity.get(), 
                            logMMAcceleration.get())
                            .withJerk(logMMJerk.get())
                            .withSlot(0)
                            .withEnableFOC(true);
    // MotionMagic type may change

    
    public enum ClimbMasterStates {
        STOWING,
        STANDBY,
        LOCKING1, //this will be removed later on
        LOCKING2, //this will be removed later on
        TapeExtend, // tape extend while elavtor retract
        ElevatorExtend //elevator extend while tape retract
    }

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

    private ClimbMasterStates currentClimbMasterStates = ClimbMasterStates.STANDBY;
    public ElevatorStates currentElevatorState = ElevatorStates.STANDBY;
    private TapeClimbStates currentTapeClimbState = TapeClimbStates.STANDBY;

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        configureMotors();
    }

    public void configureMotors() {
        //MOTOR I
        climbConfig1 = new TalonFXConfiguration();

        climbConfig1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logTapeMMKS.get())
                                        .withKV(logTapeMMKV.get())
                                        .withKA(logTapeMMKA.get())
                                        .withKP(logTapeMMKP.get())
                                        .withKI(logTapeMMKI.get())
                                        .withKD(logTapeMMKD.get());

        

        //MOTOR II

        climbConfig2 = new TalonFXConfiguration();

        climbConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot1Configs slot1Configs = new Slot1Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logElevatorMMKS.get())
                                        .withKV(logElevatorMMKV.get())
                                        .withKA(logElevatorMMKA.get())
                                        .withKP(logElevatorMMKP.get())
                                        .withKI(logElevatorMMKI.get())
                                        .withKD(logElevatorMMKD.get());

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        climbConfig1.Slot0 = slot0Configs;
        climbConfig2.Slot1 = slot1Configs;

        climbConfig1.MotionMagic = mmConfigs;
        climbConfig2.MotionMagic = mmConfigs;
        
        tapeClimbMotor.getConfigurator().apply(climbConfig1);
        elevatorClimbMotor.getConfigurator().apply(climbConfig2);
    }

    /* ----------- Updaters ----------- */

    // public void setNumberofClimbs(int numClimbs) {this.climbCounter = numClimbs;}

    public void setTarget(double setpoint) {
         tapeClimbMotor.setControl(controlRequest.withPosition(setpoint) //in rotations
                                                 .withVelocity(null)); //in rotations per second
    }

    public void resetEncoders() {
        tapeClimbMotor.setPosition(0);
        elevatorClimbMotor.setPosition(0);
    }

    private void publishClimbLogs() {
        Logger.recordOutput("Rebuilt/Climb/MotorPosition",getMotorPosition());
    }

    public void periodic() {
        applyStates();
        publishClimbLogs();
    }

    public boolean atTarget() {
        atTarget = (Math.abs(climbDistances[climbCounter].get()) - getMotorPosition() < 0.05 && tapeClimbMotor.getVelocity().getValueAsDouble() < 0.1);
        return atTarget;
    }

    // public boolean endClimbSequence() { return climbCounter == 0; }

    // public void resetStopwatch() {delayStopwatch.stop(); delayStopwatch.reset();}

    public void masterStates() {
        switch (currentClimbMasterStates) {
            case TapeExtend:
                currentElevatorState = ElevatorStates.RETRACTING;
                currentTapeClimbState = TapeClimbStates.EXTENDING;
                break;
            case ElevatorExtend:
                currentElevatorState = ElevatorStates.EXTENDING;
                currentTapeClimbState = TapeClimbStates.RETRACTING;
                break;
            case LOCKING1:
                currentTapeClimbState = TapeClimbStates.LOCKING;
                break;
            case LOCKING2:
                currentElevatorState = ElevatorStates.LOCKING;
                break;
            case STANDBY:
                break;
            case STOWING:
                break;
            default:
                break;
        }
    }

    private void applyStates() {
        //consider merging these two switches
        //stuff to consider- the motion magic configs

        //0 (climb count) + 3 = 3

        //maximum num of climbs - climb count = currentGoal
        //for instance, 3 - 2 = 1
        //therefore, 2 + 1 = 3

        // for traversing climbDistances
        // add +1 to climbCounter each time
        // stop repeating when climbCounter is greater than or maybe equal to barsToClimb

        switch (currentElevatorState) {
            case STANDBY:
                setTarget(Math.round(Math.abs(getMotorPosition())));
                //502 bad code
                break;
            case EXTENDING:
                setTarget(climbDistances[climbCounter].get());
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
                setTarget(climbDistances[climbCounter].get());
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
                setTarget(Math.round(Math.abs(getMotorPosition())));
                //make sure to set what happens during state standbyaa
                break;
            case STOWING:
                setTarget(0);
                break;
        }
    }

    public double getMotorPosition() {
        return tapeClimbMotor.getPosition().getValueAsDouble();
    }

    public static synchronized ClimberSubsystem getInstance() {
        if (climberInstance == null) {
            climberInstance = new ClimberSubsystem();
        }
        System.out.println("acquired" + climberInstance); //temporary

        return climberInstance;
    }
    
}
