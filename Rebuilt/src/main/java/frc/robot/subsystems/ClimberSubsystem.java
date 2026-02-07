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

    private TalonFX tapeClimbMotor = new TalonFX(0, RobotConstants.CANIVORE_BUS_NAME); //temporary deviceID value
    private TalonFX elevatorClimbMotor = new TalonFX(0, RobotConstants.CANIVORE_BUS_NAME); //temporary deviceID value

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
    private LoggedNetworkNumber logKG = new LoggedNetworkNumber("/Tuning/Climb/kG");

    private TalonFXConfiguration climbConfig1;
    private TalonFXConfiguration climbConfig2;
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

    public enum IHateMyLife {
        STOWING,
        STANDBY,
        LOCKING1,
        LOCKING2,
        HAND1,
        HAND2
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

    private IHateMyLife currentMasterState = IHateMyLife.STANDBY;
    private ElevatorStates currentElevatorState = ElevatorStates.STANDBY;
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
                                        .withKS(logMMKS.get())
                                        .withKV(logMMKV.get())
                                        .withKA(logMMKA.get())
                                        .withKP(logMMKP.get())
                                        .withKI(logMMKI.get())
                                        .withKD(logMMKD.get());

        

        //MOTOR II

        climbConfig2 = new TalonFXConfiguration();

        climbConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot1Configs slot1Configs = new Slot1Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(0)
                                        .withKV(0)
                                        .withKA(0)
                                        .withKP(0)
                                        .withKI(0)
                                        .withKD(0);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logMMVelocity.get())
                                                               .withMotionMagicJerk(logMMJerk.get());

        climbConfig2.Slot0 = slot0Configs;
        climbConfig1.Slot1 = slot1Configs;

        climbConfig2.MotionMagic = mmConfigs;
        climbConfig1.MotionMagic = mmConfigs;
        
        elevatorClimbMotor.getConfigurator().apply(climbConfig2);
        tapeClimbMotor.getConfigurator().apply(climbConfig1);
    }

    /* ----------- Updaters ----------- */

    public void setNumberofClimbs(int numClimbs) {
        this.climbCounter = numClimbs;
    }

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

    public boolean endClimbSequence() {
        return climbCounter == 0;
    }

    public void resetStopwatch() {
        delayStopwatch.stop();
        delayStopwatch.reset();
    }

    public void setState(int num) {
        switch (num) {
            case 1:
                currentElevatorState = ElevatorStates.STANDBY;
                break;
            case 2:
                currentElevatorState = ElevatorStates.EXTENDING;
                break;
            case 3:
                currentElevatorState = ElevatorStates.RETRACTING;
                break;
            case 4:
                currentElevatorState = ElevatorStates.LOCKING;
                break;
            case 5:
                currentElevatorState = ElevatorStates.STOWING;
                break;
        
            default:
                break;
        }
    }

    public void masterStates() {
        switch (currentMasterState) {
            case HAND1:
                currentElevatorState = ElevatorStates.RETRACTING;
                currentTapeClimbState = TapeClimbStates.EXTENDING;
                break;
            case HAND2:
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

        switch (currentElevatorState) {
            case STANDBY:
                setTarget(Math.round(Math.abs(getMotorPosition())));
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
