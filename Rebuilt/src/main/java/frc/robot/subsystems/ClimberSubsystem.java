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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ClimberConstants;
import frc.robot.Constants.RobotConstants.ClimberConstants.ClimbPositions;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;

@SuppressWarnings("unused")
public class ClimberSubsystem extends SubsystemBase {

    /* ========== INSTANCE ========= */

    private static volatile ClimberSubsystem climberInstance;

    public static synchronized ClimberSubsystem getInstance() {
        if (climberInstance == null) climberInstance = new ClimberSubsystem();
        return climberInstance;
    } 

    /* ========== MOTORS ========= */

    private TalonFX telescopeClimbMotor = new TalonFX(ClimberConstants.DeviceIDs.kTelescope, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX elevatorClimbMotor = new TalonFX(ClimberConstants.DeviceIDs.kElevator, RobotConstants.MAIN_SYSTEMS_CANBUS);

    private TalonFXConfiguration telescopeConfig;
    private TalonFXConfiguration elevatorConfig;

    /* ========== TUNING ========= */

    private LoggedNetworkNumber logTelescopeMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/Velocity", 94);
    private LoggedNetworkNumber logTelescopeMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/Acceleration", 1400);

    private LoggedNetworkNumber logTelescopeMMKS = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kS", 0.24);
    private LoggedNetworkNumber logTelescopeMMKV = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kV", 0.2);
    private LoggedNetworkNumber logTelescopeMMKA = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kA", 0);
    private LoggedNetworkNumber logTelescopeMMKP = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kP", 10);
    private LoggedNetworkNumber logTelescopeMMKI = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kI", 0.0);
    private LoggedNetworkNumber logTelescopeMMKD = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kD", 0.0);
    private LoggedNetworkNumber logTelescopeMMKG = new LoggedNetworkNumber("/Tuning/Climb/Telescope/MM/kG", 0.07);
    private double telescopingUnderLoadKG = 0.6;

    private LoggedNetworkNumber logElevatorMMVelocity = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/Velocity", 95);
    private LoggedNetworkNumber logElevatorMMAcceleration = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/Acceleration", 1200);

    private LoggedNetworkNumber logElevatorMMKS = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kS", 0.22);
    private LoggedNetworkNumber logElevatorMMKV = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kV", 0.26);
    private LoggedNetworkNumber logElevatorMMKA = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kA", 0);
    private LoggedNetworkNumber logElevatorMMKP = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kP", 5);
    private LoggedNetworkNumber logElevatorMMKI = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kI", 0.0);
    private LoggedNetworkNumber logElevatorMMKD = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kD", 0.0);
    private LoggedNetworkNumber logElevatorMMKG = new LoggedNetworkNumber("/Tuning/Climb/Elevator/MM/kG", 0.05);

    /* ========== CONTROL REQUESTS ========= */

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

    /* ========== STATES & LEVELS ========= */
    
    public enum ClimbStates {
        ZEROING,
        STANDBY,
        INIT,
        HANDOFF,
        END,
        OVERRIDE
    }

    public enum ClimbLevel {
        AUTO,
        L1,
        L2,
        L3,
    }

    private ClimbStates currentClimbState = ClimbStates.ZEROING;
    private ClimbLevel currentClimbLevel = ClimbLevel.L1;

    /* ========== VARIALBES ========= */

    private double currentElevatorSetpoint = 0;
    private double overrideElevatorSetpoint = 0;

    private double currentTelescopeSetpoint = 0;
    private double overrideTelescopeSetpoint = 0;

    private boolean isTelescopeZeroingDone = false;
    private boolean isElevatorZeroingDone = false;

    private boolean atTarget;

    private Timer zeroingTimer = new Timer();

    /* =================== INIT =================== */

    public ClimberSubsystem() {
        configureMotors();

        elevatorClimbMotor.setPosition(0);
    }

    public void configureMotors() {
        telescopeConfig = new TalonFXConfiguration();

        telescopeConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        telescopeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

        Slot1Configs telescpeSlot1Configs = new Slot1Configs()
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withKS(logTelescopeMMKS.get())
                                        .withKV(logTelescopeMMKV.get())
                                        .withKA(logTelescopeMMKA.get())
                                        .withKP(logTelescopeMMKP.get())
                                        .withKI(logTelescopeMMKI.get())
                                        .withKD(logTelescopeMMKD.get())
                                        .withKG(telescopingUnderLoadKG)
                                        .withGravityType(GravityTypeValue.Elevator_Static);

        


        elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        // elevatorConfig.CurrentLimits.StatorCurrentLimit = 10;

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

        telescopeConfig.Slot1 = telescpeSlot1Configs;

        telescopeConfig.MotionMagic = mmTelescopeConfigs;
        elevatorConfig.MotionMagic = mmElevatorConfigs;
        
        telescopeClimbMotor.getConfigurator().apply(telescopeConfig);
        elevatorClimbMotor.getConfigurator().apply(elevatorConfig);
    }

    /* =================== PERIODIC =================== */

    public void periodic() {

        if(!isElevatorZeroingDone || !isTelescopeZeroingDone) {
            currentClimbState = ClimbStates.ZEROING;
            if(!isElevatorZeroingDone) {
                if (elevatorClimbMotor.getMotorVoltage(false).getValueAsDouble() == 0 && elevatorClimbMotor.getVelocity(false).getValueAsDouble() == 0) {
                    zeroingTimer.restart();
                }
                if (zeroingTimer.get() > 0.15) {
                    isElevatorZeroingDone = elevatorAutoZeroing();
                }
            }
            if(!isTelescopeZeroingDone) {
                isTelescopeZeroingDone = telescopeAutoZeroing();
            }
        }
        applyStates();
        publishLogs();
    }

    private void applyStates() {
        switch (currentClimbState) {
            case ZEROING:
                if(!isElevatorZeroingDone) {
                    // double voltage = (-zeroingTimer.get() + 1.4) * 25;
                    // voltage = MathUtil.clamp(voltage, 0.5, 12);
                    // elevatorClimbMotor.setVoltage(voltage);
                    elevatorClimbMotor.setVoltage(0.5);
                }
                if(!isTelescopeZeroingDone) {
                    telescopeClimbMotor.setVoltage(-0.75);
                }
                if(isElevatorZeroingDone && isTelescopeZeroingDone) {
                    setClimbState(ClimbStates.STANDBY);
                }
                break;

            case STANDBY:
                elevatorClimbMotor.stopMotor();
                setTelescopeTarget(ClimbPositions.kTelescopeStore);
                break;

            case INIT:
                switch (currentClimbLevel) {
                    case AUTO:
                        setTelescopeTarget(ClimbPositions.kTelescopeReach);
                        break;

                    case L1:
                        setElevatorTarget(ClimbPositions.kElevatorStore);
                        setTelescopeTarget(ClimbPositions.kTelescopeReach);
                        break;
                    
                    case L2:
                        setElevatorTarget(ClimbPositions.kElevatorStore);
                        setTelescopeTarget(ClimbPositions.kTelescopeReach);
                        break;

                    case L3:
                        setElevatorTarget(ClimbPositions.kElevatorStore);
                        setTelescopeTarget(ClimbPositions.kTelescopeReach);
                        break;
                }
                if (ElevatorAtTarget() && TelescopeAtTarget()) {
                    if (currentClimbLevel != ClimbLevel.AUTO) {
                        // setClimbState(ClimbStates.HANDOFF);
                    } else {
                        // setClimbState(ClimbStates.END);
                    }
                }
                break;

            case HANDOFF:
                switch (currentClimbLevel) {
                    case AUTO:
                        break;

                    case L1:
                        setElevatorTarget(ClimbPositions.kElevatorHandoff);
                        setTelescopeTarget(ClimbPositions.kTelescopeHandoff);
                        break;
                    
                    case L2:
                        setElevatorTarget(ClimbPositions.kElevatorHandoff);
                        setTelescopeTarget(ClimbPositions.kTelescopeHandoff);
                        break;

                    case L3:
                        setElevatorTarget(ClimbPositions.kElevatorHandoff);
                        setTelescopeTarget(ClimbPositions.kTelescopeHandoff);
                        break;
                }
                if (ElevatorAtTarget() && TelescopeAtTarget()) {
                    if (currentClimbLevel != ClimbLevel.L3) {
                        // setClimbState(ClimbStates.END);
                    }
                }
                break;

            case END:
                switch (currentClimbLevel) {
                        case AUTO:
                            setTelescopeTarget(ClimbPositions.kTelescopeAutoClimb);
                            break;
                        case L1:
                            setElevatorTarget(ClimbPositions.kElevatorClimb);
                            if (getElevatorPosition() < -45) {
                                setTelescopeTarget(ClimbPositions.kTelescopeReach);
                            }
                            break;
                        
                        case L2:
                            setElevatorTarget(ClimbPositions.kElevatorClimb);
                            if (getElevatorPosition() < -45) {
                                setTelescopeTarget(ClimbPositions.kTelescopeReach);
                            }
                            break;

                        case L3:
                            setElevatorTarget(ClimbPositions.kElevatorL3Store);
                            setTelescopeTarget(ClimbPositions.kTelescopeStore);
                            break;
                    }
                    if (ElevatorAtTarget() && TelescopeAtTarget()) {
                        // if (currentClimbLevel == ClimbLevel.L1 || currentClimbLevel == ClimbLevel.L2) {
                        //     if (currentClimbLevel == ClimbLevel.L1) {
                        //         setClimbLevel(ClimbLevel.L2);
                        //     } else {
                        //         setClimbLevel(ClimbLevel.L3);
                        //     }
                        //     setClimbState(ClimbStates.INIT);
                        // }
                    }
                    break;

            case OVERRIDE:
                setTelescopeTarget(overrideTelescopeSetpoint);
                setElevatorTarget(overrideElevatorSetpoint);
                break;

        }
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Climb/Elevator/MotorPosition", getElevatorPosition());
        Logger.recordOutput("Rebuilt/Climb/Elevator/Setpoint", currentElevatorSetpoint);
        Logger.recordOutput("Rebuilt/Climb/Elevator/MotorVoltage", elevatorClimbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Climb/Elevator/Motor Stator Amps", elevatorClimbMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Climb/Elevator/Motor Tourqe Amps", elevatorClimbMotor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Climb/Elevator/Velocity", elevatorClimbMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Climb/Telescope/Setpoint", currentTelescopeSetpoint);
        Logger.recordOutput("Rebuilt/Climb/Telescope/MotorPosition", getTelescopePosition());
        Logger.recordOutput("Rebuilt/Climb/Telescope/MotorVoltage", telescopeClimbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Climb/Telescope/Velocity", telescopeClimbMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Climb/CurrentState", currentClimbState);
        Logger.recordOutput("Rebuilt/Climb/Level", currentClimbLevel);
    }

    /* =================== ZEROING =================== */

      public void zeroMotors() {
        telescopeClimbMotor.setPosition(0);
        elevatorClimbMotor.setPosition(0);
    }

    /**
     * TODO: needs to have fast start, and quick deceleration, but only when bumpers and V2 and made. Otherwise, keep how is.
     * @return
     */
    public boolean elevatorAutoZeroing() {
        if(Math.abs(elevatorClimbMotor.getMotorVoltage().getValueAsDouble()) > 0.3 && Math.abs(elevatorClimbMotor.getVelocity().getValueAsDouble()) < 0.08) {
            elevatorClimbMotor.setPosition(0);
            elevatorClimbMotor.setVoltage(0);

            return true;
        }
        return false;
    }

    public boolean telescopeAutoZeroing() {
        if(Math.abs(telescopeClimbMotor.getMotorVoltage().getValueAsDouble()) > 0.6 && Math.abs(telescopeClimbMotor.getVelocity().getValueAsDouble()) < 0.08) {
            telescopeClimbMotor.setPosition(0);
            telescopeClimbMotor.setVoltage(0);

            return true;
        }
        return false;
    }

    /* =================== GETTERS =================== */

    public boolean ElevatorAtTarget() {
        return ElevatorAtTarget(1.0);
    }

    public boolean ElevatorAtTarget(double tolerance) {
        atTarget = (Math.abs(currentElevatorSetpoint - getElevatorPosition()) < tolerance);
        return atTarget;
    }

    public boolean TelescopeAtTarget() {;
        return TelescopeAtTarget(0.5);
    }

    public boolean TelescopeAtTarget(double tolerance) {
        atTarget = (Math.abs(currentTelescopeSetpoint - getTelescopePosition()) < tolerance);
        return atTarget;
    }

    public double getElevatorPosition() {
        return elevatorClimbMotor.getPosition().getValueAsDouble();
    }

    public double getTelescopePosition() {
        return telescopeClimbMotor.getPosition().getValueAsDouble();
    }

    /* =================== SETTERS =================== */

    public void setClimbState(ClimbStates state) {
        if (state == ClimbStates.INIT && currentClimbLevel == ClimbLevel.L1 || state == ClimbStates.STANDBY) {
            setElevatorRequestConstants(95, 1200);
            setTelescopingRequestConstants(95, 1400);
        } else if (state == ClimbStates.END) {
            setElevatorRequestConstants(50, 200);
            setTelescopingRequestConstants(95, 200);
        } else {
            setElevatorRequestConstants(50, 200);
            setTelescopingRequestConstants(30, 200);
        }
        currentClimbState = state;
    }

    public void setClimbLevel(ClimbLevel level) {
        currentClimbLevel = level;
    }

    public void setTelescopeTarget(double setpoint) {
        currentTelescopeSetpoint = setpoint;
        if (currentClimbState == ClimbStates.HANDOFF) {
            telescopeClimbMotor.setControl(telescopeRequest.withPosition(setpoint).withSlot(1));
        } else {
            telescopeClimbMotor.setControl(telescopeRequest.withPosition(setpoint).withSlot(0)); //in rotations per second
        }
    }

    public void setElevatorTarget(double setpoint) {
        currentElevatorSetpoint = setpoint;
        elevatorClimbMotor.setControl(elevatorRequest.withPosition(setpoint));
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

    public void setElevatorRequestConstants(double veloc, double accel) {
        elevatorRequest.withVelocity(veloc).withAcceleration(accel);
    }

    public void setTelescopingRequestConstants(double veloc, double accel) {
        telescopeRequest.withVelocity(veloc).withAcceleration(accel);
    }

    /* =================== COMMANDS =================== */

    public Command GoToNextLevel() {
        return Commands.either(
                Commands.either(
                        new InstantCommand(() -> setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> setClimbState(ClimbStates.INIT))), 
                        new InstantCommand(() -> setClimbLevel(ClimbLevel.L3)).alongWith(new InstantCommand(() -> setClimbState(ClimbStates.INIT))),
                        (() -> currentClimbLevel == ClimbLevel.L1)
                    ), 
                new InstantCommand(), 
                (() -> currentClimbState == ClimbStates.END)
            );
    }

    public Command setClimbStateAndLevelCommand() {
        return Commands.either(
            new InstantCommand(() -> setClimbState(ClimbStates.INIT)),
            Commands.either(
                    new InstantCommand(() -> setClimbState(ClimbStates.STANDBY)).alongWith(new WaitCommand(0.5).alongWith(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.PREPARING_FOR_SHOOTING, true))),
                    Commands.either(
                        CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB, false).alongWith(new WaitCommand(0.4).alongWith(new InstantCommand(() -> setClimbLevel(ClimbLevel.L1)).alongWith(new InstantCommand(() -> setClimbState(ClimbStates.INIT))))),
                        Commands.either(
                            Commands.either(
                                new InstantCommand(() -> setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> setClimbState(ClimbStates.INIT))), 
                                new InstantCommand(() -> setClimbLevel(ClimbLevel.L3)).alongWith(new InstantCommand(() -> setClimbState(ClimbStates.INIT))),
                                (() -> currentClimbLevel == ClimbLevel.L1)
                                ), 
                            Commands.either(
                                new InstantCommand(() -> setClimbState(ClimbStates.HANDOFF)), 
                                new InstantCommand(() -> setClimbState(ClimbStates.END)), 
                                (() -> currentClimbState == ClimbStates.INIT)
                                ), 
                            (() -> currentClimbState == ClimbStates.END)
                        ),
                        (() -> currentClimbState == ClimbStates.STANDBY)
                    ),
                    (() -> currentClimbState == ClimbStates.INIT && currentClimbLevel == ClimbLevel.AUTO)
            ),
            (() -> currentClimbState == ClimbStates.END && currentClimbLevel == ClimbLevel.AUTO)
        );
    }

    public Command setForceClimberUpCommand() {
        return new InstantCommand(() -> setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> setClimbState(ClimbStates.INIT)));
    }

    public Command setForceClimberDownCommand() {
        return new InstantCommand(() -> setClimbState(ClimbStates.STANDBY));
    }

    /* =================== UPDATER/TUNING =================== */    

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
}