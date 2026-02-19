package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ClimberConstants;
import frc.robot.Constants.RobotConstants.ClimberConstants.ClimbPositions;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.ShooterSubsystem.AngleState;

public class ClimberSubsystem extends SubsystemBase {

    private static volatile ClimberSubsystem climberInstance;

    private TalonFX telescopeClimbMotor = new TalonFX(ClimberConstants.DeviceIDs.kTelescope, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value
    private TalonFX elevatorClimbMotor = new TalonFX(ClimberConstants.DeviceIDs.kElevator, RobotConstants.MAIN_SYSTEMS_CANBUS); //temporary deviceID value

    // for motor 1/ telescope motor
    private LoggedNetworkNumber logTelescopeMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/Velocity", 94);
    private LoggedNetworkNumber logTelescopeMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/Acceleration", 1400); //Needs a slower one for way down
    private double telescopeMMVeloc = logTelescopeMMVelocity.get();
    private double telescopeMMAccel = logTelescopeMMAcceleration.get();

    private LoggedNetworkNumber logTelescopeMMKS = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kS", 0.24);
    private LoggedNetworkNumber logTelescopeMMKV = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kV", 0.2);
    private LoggedNetworkNumber logTelescopeMMKA = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kA", 0);
    private LoggedNetworkNumber logTelescopeMMKP = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kP", 5);
    private LoggedNetworkNumber logTelescopeMMKI = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kI", 0.0);
    private LoggedNetworkNumber logTelescopeMMKD = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kD", 0.0);
    private LoggedNetworkNumber logTelescopeMMKG = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kG", 0.07);

    // for motor 2/ elevator motor
    private LoggedNetworkNumber logElevatorMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/Velocity", 95);
    private LoggedNetworkNumber logElevatorMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/Acceleration", 1200);

    private LoggedNetworkNumber logElevatorMMKS = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kS", 0.22);
    private LoggedNetworkNumber logElevatorMMKV = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kV", 0.26);
    private LoggedNetworkNumber logElevatorMMKA = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kA", 0);
    private LoggedNetworkNumber logElevatorMMKP = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kP", 8);
    private LoggedNetworkNumber logElevatorMMKI = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kI", 0.0);
    private LoggedNetworkNumber logElevatorMMKD = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kD", 0.0);
    private LoggedNetworkNumber logElevatorMMKG = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kG", 0.05);

    // climbDistances holds a position value for the motor to go to for each bar
    private LoggedNetworkNumber[] climbDistances = 
    {  
        new LoggedNetworkNumber("/Tuning/Climb/L3Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L2Climb", 0.0),
        new LoggedNetworkNumber("/Tuning/Climb/L1Climb", 0.0)
    };

    // Input from the user before the match to say what climb they want to do
    private LoggedNetworkNumber barsToClimb = new LoggedNetworkNumber("/Tuning/Climb/barsToClimb", 1);

    private TalonFXConfiguration telescopeConfig;
    private TalonFXConfiguration elevatorConfig;
    boolean atTarget;

    // index for the list climbDistance
    private int climbCounter = 0;

    private DynamicMotionMagicVoltage elevatorRequest = 
                        new DynamicMotionMagicVoltage(0, 
                            logElevatorMMVelocity.get(), 
                            logElevatorMMAcceleration.get())
                            .withSlot(0)
                            .withEnableFOC(true);

    private DynamicMotionMagicVoltage telescopeRequest = 
                        new DynamicMotionMagicVoltage(0, 
                            logTelescopeMMVelocity.get(), 
                            logTelescopeMMAcceleration.get())
                            .withSlot(0)
                            .withEnableFOC(true);
    
    public enum ClimbMasterStates {
        ZEROING,
        //STOWING, // may not be needed
        STANDBY,
        // LOCKING1, // may not be needed
        // LOCKING2, // may not be needed
        L1_START,
        L1_CLIMED,
        L2_START,
        L2_CLIMBED,
        L3_START,
        L3_CLIMED,
        OVERRIDE
        // -------------- vvv I'm not sure at this point how the elevator and telescope work 
        // together because the climber was finished on winter break
        
        // REACHING, // Telescope extend while elavtor retract
        // PULLING //elevator extend while Telescope retract
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

    private ClimbMasterStates currentClimbMasterStates = ClimbMasterStates.ZEROING;
    // public ElevatorStates currentElevatorState = ElevatorStates.STANDBY;
    private double currentElevatorSetpoint = 0;
    private double overrideElevatorSetpoint = 0; // Min is -128.35 if zeroing at top
;
    // private TelescopeStates currentTelescopeState = TelescopeStates.STANDBY;
    private double currentTelescopeSetpoint = 0;
    private double overrideTelescopeSetpoint = 0; // Max is 40

    private boolean L3Climb = false;
    private boolean isTelescopeZeroingDone = false;
    private boolean isElevatorZeroingDone = false;
    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        configureMotors();
    }

    public void configureMotors() {
        //MOTOR I
        telescopeConfig = new TalonFXConfiguration();

        telescopeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        telescopeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // telescopeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // telescopeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        // telescopeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40; // Hood angle limit
        // telescopeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        Slot0Configs telescopeSlot0Configs = new Slot0Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logTelescopeMMKS.get())
                                        .withKV(logTelescopeMMKV.get())
                                        .withKA(logTelescopeMMKA.get())
                                        .withKP(logTelescopeMMKP.get())
                                        .withKI(logTelescopeMMKI.get())
                                        .withKD(logTelescopeMMKD.get())
                                        .withKG(logTelescopeMMKG.get())
                                        .withGravityType(GravityTypeValue.Elevator_Static);

        

        //MOTOR II

        elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs elevatorSlot0Configs = new Slot0Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logElevatorMMKS.get())
                                        .withKV(logElevatorMMKV.get())
                                        .withKA(logElevatorMMKA.get())
                                        .withKP(logElevatorMMKP.get())
                                        .withKI(logElevatorMMKI.get())
                                        .withKD(logElevatorMMKD.get())
                                        .withKG(logElevatorMMKG.get())
                                        .withGravityType(GravityTypeValue.Elevator_Static);

        MotionMagicConfigs mmElevatorConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logElevatorMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logElevatorMMVelocity.get());

        MotionMagicConfigs mmTelescopeConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logTelescopeMMAcceleration.get())
                                                               .withMotionMagicCruiseVelocity(logTelescopeMMVelocity.get());

        telescopeConfig.Slot0 = telescopeSlot0Configs;
        elevatorConfig.Slot0 = elevatorSlot0Configs;

        telescopeConfig.MotionMagic = mmTelescopeConfigs;
        elevatorConfig.MotionMagic = mmElevatorConfigs;
        
        telescopeClimbMotor.getConfigurator().apply(telescopeConfig);
        elevatorClimbMotor.getConfigurator().apply(elevatorConfig);
    }

    /* ----------- Updaters ----------- */

    public void setTelescopeTarget(double setpoint) {
        currentTelescopeSetpoint = setpoint;
        telescopeClimbMotor.setControl(telescopeRequest.withPosition(setpoint)); //in rotations per second
    }

    public void setElevatorTarget(double setpoint) {
        currentElevatorSetpoint = setpoint;
        elevatorClimbMotor.setControl(elevatorRequest.withPosition(setpoint));
    }

    public void setClimbState(ClimbMasterStates state) {
        currentClimbMasterStates = state;
    }

    public void setOverrideSetpoints(double elevatorSetpoint, double telescopeSetpoint) {
        overrideElevatorSetpoint = elevatorSetpoint;
        overrideTelescopeSetpoint = telescopeSetpoint;
    }

    public void setElevatorOverrideSetpoint(double elevatorSetpoint) {
        overrideElevatorSetpoint = elevatorSetpoint;
    }

    public void setTelescopeOverrideSetpoint(double telescopeSetpoint) {
        overrideTelescopeSetpoint = telescopeSetpoint;
    }

    public void zeroMotors() {
        telescopeClimbMotor.setPosition(0);
        elevatorClimbMotor.setPosition(0);
    }

    public boolean elevatorAutoZeroing() {
        if(Math.abs(elevatorClimbMotor.getMotorVoltage().getValueAsDouble()) > 0.3 && Math.abs(elevatorClimbMotor.getVelocity().getValueAsDouble()) < 0.02) {
            elevatorClimbMotor.setPosition(0);
            elevatorClimbMotor.setVoltage(0);

            return true;
        }
        return false;
    }

    public boolean telescopeAutoZeroing() {
        if(Math.abs(telescopeClimbMotor.getMotorVoltage().getValueAsDouble()) > 0.3 && Math.abs(telescopeClimbMotor.getVelocity().getValueAsDouble()) < 0.02) {
            telescopeClimbMotor.setPosition(0);
            telescopeClimbMotor.setVoltage(0);

            return true;
        }
        return false;
    }

    private void publishClimbLogs() {
        Logger.recordOutput("Rebuilt/Climb/Elevator/MotorPosition", getElevatorPosition());
        Logger.recordOutput("Rebuilt/Climb/Elevator/Setpoint", currentElevatorSetpoint);
        Logger.recordOutput("Rebuilt/Climb/Elevator/MotorVoltage", elevatorClimbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Climb/Elevator/Velocity", elevatorClimbMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Climb/Telescope/Setpoint", currentTelescopeSetpoint);
        Logger.recordOutput("Rebuilt/Climb/Telescope/MotorPosition", getTelescopePosition());
        Logger.recordOutput("Rebuilt/Climb/Telescope/MotorVoltage", telescopeClimbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Climb/Telescope/Velocity", telescopeClimbMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Climb/CurrentState", currentClimbMasterStates);
    }

    public void periodic() {

        if(!isElevatorZeroingDone || !isTelescopeZeroingDone) {
            currentClimbMasterStates = ClimbMasterStates.ZEROING;
            if(!isElevatorZeroingDone) {
                isElevatorZeroingDone = elevatorAutoZeroing();
            }
            if(!isTelescopeZeroingDone) {
                isTelescopeZeroingDone = telescopeAutoZeroing();
            }
        }
        applyStates();
        publishClimbLogs();
    }

    public boolean ElevatorAtTarget() {
        atTarget = (Math.abs(currentElevatorSetpoint - getElevatorPosition()) < 1.5);
        return false;
        // return atTarget;
    }

    public boolean ElevatorAtTarget(double position) {
        atTarget = (Math.abs(position) - getElevatorPosition() < 1.5);
        return atTarget;
    }

    public boolean TelescopeAtTarget() {
        atTarget = (Math.abs(currentTelescopeSetpoint - getTelescopePosition()) < 0.05 && telescopeClimbMotor.getVelocity().getValueAsDouble() < 0.1);
        // return atTarget;
        return false;
    }

    public boolean TelescopeAtTarget(double position) {
        atTarget = (Math.abs(position) - getTelescopePosition() < 0.05 && telescopeClimbMotor.getVelocity().getValueAsDouble() < 0.1);
        return atTarget;
    }

    private void applyStates() {

        // ------------ The state Locking and Stowing may not be useful and could be removed ----
        switch (currentClimbMasterStates) {
            case ZEROING:
                if(!isElevatorZeroingDone) {
                    elevatorClimbMotor.setVoltage(.7);
                }
                if(!isTelescopeZeroingDone) {
                    telescopeClimbMotor.setVoltage(-.7);
                }
                if(isElevatorZeroingDone && isTelescopeZeroingDone) {
                    setClimbState(ClimbMasterStates.OVERRIDE);
                }
                break;

            case STANDBY:
                setElevatorTarget(0);
                setTelescopeTarget(0);
                break;

            case L1_START:
                setElevatorTarget(ClimbPositions.kElevatorFullyUp);
                setTelescopeTarget(ClimbPositions.kTelescopeFullyUp);
                if(ElevatorAtTarget() && TelescopeAtTarget()) {
                    currentClimbMasterStates = ClimbMasterStates.L1_CLIMED;
                }
                break;

            case L1_CLIMED:
                setTelescopeTarget(0);
                telescopeConfig.Slot0.withKG(3); //temporary solution
                setElevatorTarget(ClimbPositions.kElevatorFullyUp);
                if(ElevatorAtTarget() && TelescopeAtTarget() && L3Climb) { //will only climb to l2 if it is an l3 climb?
                    currentClimbMasterStates = ClimbMasterStates.L2_START;
                }
                break;

            case L2_START:
                setElevatorTarget(0);
                elevatorConfig.Slot0.withKG(3); //"there is nothing more permanent than a temporary solution" -Russian proverb
                setTelescopeTarget(ClimbPositions.kTelescopeFullyUp);
                telescopeConfig.Slot0.withKG(logTelescopeMMKG.get()); //temporary solution
                if(ElevatorAtTarget() && TelescopeAtTarget() && L3Climb) {
                    currentClimbMasterStates = ClimbMasterStates.L2_CLIMBED;
                }
                break;

            case L2_CLIMBED:
                setTelescopeTarget(0);
                telescopeConfig.Slot0.withKG(3); //temporary solution
                setElevatorTarget(ClimbPositions.kElevatorFullyUp);
                if(ElevatorAtTarget() && TelescopeAtTarget() && L3Climb) {
                    currentClimbMasterStates = ClimbMasterStates.L3_START;
                }
                break;

            case L3_START:
                setTelescopeTarget(ClimbPositions.kTelescopeFullyUp);
                setElevatorTarget(0);
                if(ElevatorAtTarget() && TelescopeAtTarget() && L3Climb) {
                    currentClimbMasterStates = ClimbMasterStates.L3_CLIMED;
                }
                break;

            case L3_CLIMED:
                setTelescopeTarget(ClimbPositions.kTelescopeFullyUp);
                setElevatorTarget(0);
                if(ElevatorAtTarget() && TelescopeAtTarget() && L3Climb) {
                    currentClimbMasterStates = ClimbMasterStates.L3_CLIMED;
                }
                break;
            
            case OVERRIDE:
                setTelescopeTarget(overrideTelescopeSetpoint);
                setElevatorTarget(overrideElevatorSetpoint);
                break;

        }
        
    }

    public double getElevatorPosition() {
        return elevatorClimbMotor.getPosition().getValueAsDouble();
    }

    public double getTelescopePosition() {
        return telescopeClimbMotor.getPosition().getValueAsDouble();
    }

    public void updateClimberConstants() {
            boolean updateElevatorVals = false;
            boolean updateTelescopeVals = false;

            updateTelescopeVals = (telescopeConfig.Slot0.kS != logTelescopeMMKS.get())
                        || (telescopeConfig.Slot0.kV != logTelescopeMMKV.get())
                        || (telescopeConfig.Slot0.kA != logTelescopeMMKA.get())
                        || (telescopeConfig.Slot0.kP != logTelescopeMMKP.get())
                        || (telescopeConfig.Slot0.kI != logTelescopeMMKI.get())
                        || (telescopeConfig.Slot0.kD != logTelescopeMMKD.get())
                        || (telescopeConfig.Slot0.kG != logTelescopeMMKG.get())
                        || (telescopeRequest.Velocity != logTelescopeMMVelocity.get())
                        || (telescopeRequest.Acceleration!= logTelescopeMMAcceleration.get());

            if (updateTelescopeVals) {
                telescopeConfig.Slot0.kS = logTelescopeMMKS.get();
                telescopeConfig.Slot0.kV = logTelescopeMMKV.get();
                telescopeConfig.Slot0.kA = logTelescopeMMKA.get();
                telescopeConfig.Slot0.kP = logTelescopeMMKP.get();
                telescopeConfig.Slot0.kI = logTelescopeMMKI.get();
                telescopeConfig.Slot0.kD = logTelescopeMMKD.get();
                telescopeConfig.Slot0.kG = logTelescopeMMKG.get();
                configureMotors();

                // telescopeRequest = new DynamicMotionMagicVoltage(0, logTelescopeMMVelocity.get(), logTelescopeMMAcceleration.get());//.withEnableFOC(RobotConstants.ENABLE_FOC);
            }
            
            updateElevatorVals = (elevatorConfig.Slot0.kS != logElevatorMMKS.get())
                        || (elevatorConfig.Slot0.kV != logElevatorMMKV.get())
                        || (elevatorConfig.Slot0.kA != logElevatorMMKA.get())
                        || (elevatorConfig.Slot0.kP != logElevatorMMKP.get())
                        || (elevatorConfig.Slot0.kI != logElevatorMMKI.get())
                        || (elevatorConfig.Slot0.kD != logElevatorMMKD.get())
                        || (elevatorConfig.Slot0.kG != logElevatorMMKG.get())
                        || (elevatorRequest.Velocity != logElevatorMMVelocity.get())
                        || (elevatorRequest.Acceleration != logElevatorMMAcceleration.get());;

            if (updateElevatorVals) {
                elevatorConfig.Slot0.kS = logElevatorMMKS.get();
                elevatorConfig.Slot0.kV = logElevatorMMKV.get();
                elevatorConfig.Slot0.kA = logElevatorMMKA.get();
                elevatorConfig.Slot0.kP = logElevatorMMKP.get();
                elevatorConfig.Slot0.kI = logElevatorMMKI.get();
                elevatorConfig.Slot0.kD = logElevatorMMKD.get();
                elevatorConfig.Slot0.kG = logElevatorMMKG.get();
                configureMotors();


                // elevatorRequest = new DynamicMotionMagicVoltage(0, logElevatorMMVelocity.get(), logElevatorMMAcceleration.get());//.withEnableFOC(RobotConstants.ENABLE_FOC);
            }
            
        }

    public void setElevatorRequestConstants(double veloc, double accel) {
        elevatorRequest.withVelocity(veloc).withAcceleration(accel);
    }

    public void setTelescopingRequestConstants(double veloc, double accel) {
        telescopeRequest.withVelocity(veloc).withAcceleration(accel);
    }

    public static synchronized ClimberSubsystem getInstance() {
        if (climberInstance == null) {
            climberInstance = new ClimberSubsystem();
        }
        System.out.println("acquired" + climberInstance); //temporary

        return climberInstance;
    } 

}