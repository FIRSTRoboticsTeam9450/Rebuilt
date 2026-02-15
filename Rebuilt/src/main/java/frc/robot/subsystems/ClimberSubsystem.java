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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

    private static volatile ClimberSubsystem climberInstance;

    private TalonFX telescopeClimbMotor = new TalonFX(0, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value
    private TalonFX elevatorClimbMotor = new TalonFX(0, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value

    private LoggedNetworkNumber logMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/MM/Velocity", 0.1);
    private LoggedNetworkNumber logMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/MM/Acceleration", 1);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Climb/MM/Jerk", 1000);
    
    // for motor 1/ telescope motor
    private LoggedNetworkNumber logTelescopeMMKS = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kS", 0.24);
    private LoggedNetworkNumber logTelescopeMMKV = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kV", 0.33);
    private LoggedNetworkNumber logTelescopeMMKA = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kA", 0.1);
    private LoggedNetworkNumber logTelescopeMMKP = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kP", 0.5);
    private LoggedNetworkNumber logTelescopeMMKI = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kI", 0.0);
    private LoggedNetworkNumber logTelescopeMMKD = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kD", 0.0);

    // for motor 2/ elevator motor
    private LoggedNetworkNumber logElevatorMMKS = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kS", 0.24);
    private LoggedNetworkNumber logElevatorMMKV = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kV", 0.33);
    private LoggedNetworkNumber logElevatorMMKA = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kA", 0.1);
    private LoggedNetworkNumber logElevatorMMKP = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kP", 0.5);
    private LoggedNetworkNumber logElevatorMMKI = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kI", 0.0);
    private LoggedNetworkNumber logElevatorMMKD = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kD", 0.0);

    // climbDistances holds a position value for the motor to go to for each bar
    private LoggedNetworkNumber[] climbDistances = 
    {  
        new LoggedNetworkNumber("/Tuning/Climb/L3Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L2Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L1Climb", 0.0)
    };

    // Input from the user before the match to say what climb they want to do
    private LoggedNetworkNumber barsToClimb = new LoggedNetworkNumber("/Tuning/Climb/barsToClimb", 1);

    private TalonFXConfiguration climbConfig1;
    private TalonFXConfiguration climbConfig2;
    boolean atTarget;

    // index for the list climbDistance
    private int climbCounter = 0;

    private DynamicMotionMagicVoltage controlRequest = 
                        new DynamicMotionMagicVoltage(0, 
                            logMMVelocity.get(), 
                            logMMAcceleration.get())
                            .withJerk(logMMJerk.get())
                            .withSlot(0)
                            .withEnableFOC(true);
    
    public enum ClimbMasterStates {
        STOWING, // may not be needed
        STANDBY,
        LOCKING1, // may not be needed
        LOCKING2, // may not be needed
        
        // -------------- vvv I'm not sure at this point how the elevator and telescope work 
        // together because the climber was finished on winter break
        
        TelescopeExtend, // Telescope extend while elavtor retract
        ElevatorExtend //elevator extend while Telescope retract
    }

    public enum ElevatorStates {
        STOWING, // may not be needed
        STANDBY,
        EXTENDING,
        LOCKING, // may not be needed
        RETRACTING
    }

    public enum TelescopeStates {
        STOWING, // may not be needed
        STANDBY,
        EXTENDING,
        LOCKING, // may not be needed
        RETRACTING
    }

    private ClimbMasterStates currentClimbMasterStates = ClimbMasterStates.STANDBY;
    public ElevatorStates currentElevatorState = ElevatorStates.STANDBY;
    private TelescopeStates currentTelescopeState = TelescopeStates.STANDBY;

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
                                        .withKS(logTelescopeMMKS.get())
                                        .withKV(logTelescopeMMKV.get())
                                        .withKA(logTelescopeMMKA.get())
                                        .withKP(logTelescopeMMKP.get())
                                        .withKI(logTelescopeMMKI.get())
                                        .withKD(logTelescopeMMKD.get());

        

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
        
        telescopeClimbMotor.getConfigurator().apply(climbConfig1);
        elevatorClimbMotor.getConfigurator().apply(climbConfig2);
    }

    /* ----------- Updaters ----------- */

    public void setTelescopeTarget(double setpoint) {
         telescopeClimbMotor.setControl(controlRequest.withPosition(setpoint) //in rotations
                                                 .withVelocity(null)); //in rotations per second
    }

    public void setElevatorTarget(double setpoint) {
        elevatorClimbMotor.setControl(controlRequest.withPosition(setpoint)
                                                    .withVelocity(null));
    }

    public void resetEncoders() {
        telescopeClimbMotor.setPosition(0);
        elevatorClimbMotor.setPosition(0);
    }

    private void publishClimbLogs() {
        Logger.recordOutput("Rebuilt/Climb/ElevatorMotorPosition", getElevatorPosition());
        Logger.recordOutput("Rebuilt/Climb/TelescopeMotorPosition", getTelescopePosition());
    }

    public void periodic() {
        applyStates();
        publishClimbLogs();
    }

    public boolean ElevatorAtTarget() {
        atTarget = (Math.abs(climbDistances[climbCounter].get()) - getElevatorPosition() < 0.05 && elevatorClimbMotor.getVelocity().getValueAsDouble() < 0.1);
        return atTarget;
    }

    public boolean TelescopeAtTarget() {
        atTarget = (Math.abs(climbDistances[climbCounter].get()) - getTelescopePosition() < 0.05 && telescopeClimbMotor.getVelocity().getValueAsDouble() < 0.1);
        return atTarget;
    }

    // public boolean endClimbSequence() { return climbCounter == 0; }

    public void masterStates() {
        switch (currentClimbMasterStates) {
            case TelescopeExtend:
                currentElevatorState = ElevatorStates.RETRACTING;
                currentTelescopeState = TelescopeStates.EXTENDING;
                break;
            case ElevatorExtend:
                currentElevatorState = ElevatorStates.EXTENDING;
                currentTelescopeState = TelescopeStates.RETRACTING;
                break;
            case LOCKING1:
                currentTelescopeState = TelescopeStates.LOCKING;
                break;
            case LOCKING2:
                currentElevatorState = ElevatorStates.LOCKING;
                break;
            case STANDBY:
                break;
            case STOWING:
                currentElevatorState = ElevatorStates.STOWING;
                currentTelescopeState = TelescopeStates.STOWING;
                break;
            default:
                break;
        }
    }
    

    private void applyStates() {
        // for traversing climbDistances the list
        // add +1 to climbCounter each time
        // stop repeating when climbCounter is greater than or maybe equal to barsToClimb

        // climbDistances holds a position value for the motor to go to for each bar

        // ------------ The state Locking and Stowing may not be useful and could be removed ----

        switch (currentElevatorState) {
            case STANDBY:
                // Makes the motor stay at it's current position there is probably a better way to do this
                setElevatorTarget(Math.round(Math.abs(getElevatorPosition())));
                break;
            case EXTENDING:
                setElevatorTarget(climbDistances[climbCounter].get());
                // if(ElevatorAtTarget()) {
                //     currentElevatorState = ElevatorStates.LOCKING;
                // }
                break;
            case LOCKING:
                setElevatorTarget(5); //temporary value
                // if(ElevatorAtTarget()) {
                //     currentElevatorState = ElevatorStates.RETRACTING;
                // }
                break;
            case RETRACTING:
                setElevatorTarget(0);
                // if(ElevatorAtTarget()) {
                //     currentElevatorState = ElevatorStates.STANDBY;
                // }
                break;
            case STOWING:
                setElevatorTarget(0);
                // if(ElevatorAtTarget()) {
                //     currentElevatorState = ElevatorStates.STANDBY;
                // } 
                break;
            default:
                break;
        }
        switch(currentTelescopeState) {
            case STANDBY:
                // Makes the motor stay at it's current position there is probably a better way to do this
                setTelescopeTarget(Math.round(Math.abs(getTelescopePosition())));
                break;
            case EXTENDING:
                setTelescopeTarget(climbDistances[climbCounter].get());
                if(TelescopeAtTarget()) {
                    currentTelescopeState = TelescopeStates.LOCKING;
                }
                break;
            case LOCKING:
                setTelescopeTarget(5);
                // if(TelescopeAtTarget()) {
                //     currentTelescopeState = TelescopeStates.RETRACTING;
                // }
                break;
            case RETRACTING:
                setTelescopeTarget(0);
                // if(TelescopeAtTarget()) {
                //     currentTelescopeState = TelescopeStates.STANDBY;
                // }
                break;
            case STOWING:
                setTelescopeTarget(0);
                break;
            default:
                break;
        } 
    }


    public double getElevatorPosition() {
        return elevatorClimbMotor.getPosition().getValueAsDouble();
    }

    public double getTelescopePosition() {
        return telescopeClimbMotor.getPosition().getValueAsDouble();
    }

    public static synchronized ClimberSubsystem getInstance() {
        if (climberInstance == null) {
            climberInstance = new ClimberSubsystem();
        }
        System.out.println("acquired" + climberInstance); //temporary

        return climberInstance;
    } 

}