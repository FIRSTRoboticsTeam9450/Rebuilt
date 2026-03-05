package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeSubsystem extends SubsystemBase{

    private static volatile IntakeSubsystem INSTACNE;

    //List the states the intake will use
    public enum IntakePos {
        SYS_ID,
        ZEROING,
        STORE,
        CLIMB_STORE,
        TRENCH_STORE,
        INTAKING
    }

    public enum IntakeState {
        IDLING,
        SLOW_INTAKING,
        INTAKING,
    }

    //Set the current state of the robot to be idle
    private IntakePos currentPos = IntakePos.ZEROING;
    private IntakeState currentState = IntakeState.IDLING;
    
    //Set up motors
    TalonFX motorArm = new TalonFX(IntakeConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    TalonFX motorWheels = new TalonFX(IntakeConstants.DeviceIDs.kWheelMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Intake/Angle/Velocity", 45); //45
    private LoggedNetworkNumber logMMAccel = new LoggedNetworkNumber("/Tuning/Intake/Angle/Acceleration", 100); //30


    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Intake/Angle/kS", 0.145); // .1
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Intake/Angle/kV", 0.2); // .33
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Intake/Angle/kA", 0); // 0
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Intake/Angle/kP", 30); // 25
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Intake/Angle/kI", 0); // 0
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Intake/Angle/kD", 0); // 0
    private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Intake/Angle/kG", 0); // 0

    private LoggedNetworkNumber logIntakeTarget = new LoggedNetworkNumber("/Tuning/Intake/Angle/Intaking Target", 0.9);
    private LoggedNetworkNumber logIntakingTarget = new LoggedNetworkNumber("/Tuning/Intake/Wheels/Target", 12);

    private double intakingTargetVolts = logIntakingTarget.get();
    
    private double intakeRevolutions = logIntakeTarget.get();
    private double storeRevolutions = 6.25;
    private double trenchStoreRevolutions = 4.5;
    private double climbStoreRevolutions = 7.5;

    //Motion Magic Set Up(Intake arm)
    private MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private boolean isZeroing = false;

    private boolean reversing = false;

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotionMagic.MotionMagicAcceleration = logMMAccel.get();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = logMMVeloc.get();

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 5;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        //Magic Motion
        var slot0Configs = motorConfig.Slot0;
        slot0Configs.kS = logMMKS.get();
        slot0Configs.kV = logMMKV.get();
        slot0Configs.kA = logMMKA.get();
        slot0Configs.kP = logMMKP.get();
        slot0Configs.kI = logMMKI.get();
        slot0Configs.kD = logMMKD.get();
        slot0Configs.kG = logMMKG.get();
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        motorArm.getConfigurator().apply(motorConfig);

        //Configure intake motors
        TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();
        IntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = 80;
        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = 80;
        IntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        motorWheels.getConfigurator().apply(IntakeConfig);
    }

    public boolean isZeroingDone() {
        if(Math.abs(motorArm.getVelocity().getValueAsDouble()) < .05 && Math.abs(motorArm.getMotorVoltage().getValueAsDouble()) > .8) {
            motorArm.setPosition(0);
            motorArm.setVoltage(0);
            currentPos = IntakePos.STORE;
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        if(!isZeroing) {
            currentPos = IntakePos.ZEROING;
            isZeroing = isZeroingDone();
        }
        applyState();
        publishLogs();
    }

    private void applyState() {
        switch (currentPos) {
            case SYS_ID:
                break;

            case ZEROING:
                motorArm.setVoltage(-1.3);
                break;

            case TRENCH_STORE:
                motorArm.setControl(request.withPosition(trenchStoreRevolutions));
                break;

            case STORE:
                motorArm.setControl(request.withPosition(storeRevolutions));
                break;

            case CLIMB_STORE:
                motorArm.setControl(request.withPosition(climbStoreRevolutions));
                break;
            
            case INTAKING:
                    motorArm.setControl(request.withPosition(intakeRevolutions));
                break;

            default:
                motorArm.setControl(request.withPosition(storeRevolutions));
                break;
        }

        switch (currentState) {
            case IDLING:
                motorWheels.setControl(voltageRequest.withOutput(0));
                break;

            case SLOW_INTAKING:
                motorWheels.setControl(voltageRequest.withOutput(intakingTargetVolts * 0.833333)); //0.58333
                break;
            
            case INTAKING:
            
                    // if (hasReachedIntakeLocation && motorWheels.getMotorVoltage().getValueAsDouble() > 5 && motorWheels.getVelocity().getValueAsDouble() < 75) {
                //     motorWheels.setControl(voltageRequest.withOutput(-logIntakingTarget.get()));
                // } else {
                // else if(checkJam()) {
                //     motorWheels.setControl(voltageRequest.withOutput(-1));
                //     if(!unjamTimerStarted) {
                //         unjamTimerStarted = true;
                //         unjamTimer.restart();
                //     }
                //     if(unjamTimer.get() > 1) {
                //         unjammed = true;
                //         unjamTimerStarted = false;
                //         jamTimerStarted = false;
                //     }
                // }
                // else {
                if(reversing) {
                    intakingTargetVolts = -intakingTargetVolts;
                }
                    motorWheels.setControl(voltageRequest.withOutput(intakingTargetVolts));
                // }
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Intake/Angle/Current Pos", currentPos);

        Logger.recordOutput("Rebuilt/Intake/Angle/Position", motorArm.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Target", currentPos == IntakePos.INTAKING ? logIntakeTarget.get() : storeRevolutions);
        Logger.recordOutput("Rebuilt/Intake/Angle/Motor Voltage", motorArm.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Motor Velocity", motorArm.getVelocity().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Intake/Wheels/Motor Veloc", motorWheels.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Motor Voltage", motorWheels.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Temp", motorArm.getDeviceTemp().getValueAsDouble());
    }

    public IntakePos getCurrentIntakePos() {
        return currentPos;
    }
    public void setPos(IntakePos pos) {
        currentPos = pos;
    }

    public void setState(IntakeState state) {
        currentState = state;
    }

    public Command setPosAndStateCommand(IntakePos pos, IntakeState state) {
        return new InstantCommand(() -> {setPos(pos); setState(state);});
    }

    public static synchronized IntakeSubsystem getInstance() {
        if (INSTACNE == null) INSTACNE = new IntakeSubsystem();
        return INSTACNE;
    }

    public void updateConstants() {
        boolean gainsChanged = false;

        if (motorConfig.MotionMagic.MotionMagicAcceleration != logMMAccel.get() ||
            motorConfig.MotionMagic.MotionMagicCruiseVelocity != logMMVeloc.get() ||
            motorConfig.Slot0.kS != logMMKS.get() ||
            motorConfig.Slot0.kV != logMMKV.get() ||
            motorConfig.Slot0.kA != logMMKA.get() ||
            motorConfig.Slot0.kP != logMMKP.get() ||
            motorConfig.Slot0.kI != logMMKI.get() ||
            motorConfig.Slot0.kD != logMMKD.get() ||
            motorConfig.Slot0.kG != logMMKG.get()
            ) {
            
            gainsChanged = true;
        }

        if (gainsChanged) {
            configureMotors();
        }

        intakeRevolutions = logIntakeTarget.get();
        intakingTargetVolts = logIntakingTarget.get();
    }

    public void zeroEncoders() {
        if (DriverStation.isDisabled()) {
            motorArm.setPosition(0);
        }
    }
}