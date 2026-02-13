package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.SignalLogger;
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
        TRENCH_STORE,
        INTAKING
    }

    public enum IntakeState {
        IDLING,
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

    private LoggedNetworkNumber logIntakeTarget = new LoggedNetworkNumber("/Tuning/Intake/Angle/Intaking Target", 0.73);
    private LoggedNetworkNumber logIntakingTarget = new LoggedNetworkNumber("/Tuning/Intake/Wheels/Target", 12);

    private double storeRevolutions = 6.25;
    private double trenchStoreRevolutions = 4.5;

    private boolean hasReachedIntakeLocation = false;

    //Motion Magic Set Up(Intake arm)
    private MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private final VoltageOut sysIDRequest = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine;

    private boolean isZeroing = false;

    public IntakeSubsystem() {
        configureMotors();

        //sys id
        m_sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volt.of(1), // Reduce dynamic step voltage to 1 to prevent brownout
                    null,        // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) -> motorArm.setControl(sysIDRequest.withOutput(volts.in(Volt))),
                    null,
                    this
                )
            );
    }

    private void configureMotors() {
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotionMagic.MotionMagicAcceleration = logMMAccel.get();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = logMMVeloc.get();

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

        motorWheels.getConfigurator().apply(IntakeConfig);
    }

    public boolean isZeroingDone() {
        if(Math.abs(motorArm.getVelocity().getValueAsDouble()) < .05 && Math.abs(motorArm.getMotorVoltage().getValueAsDouble()) > .4) {
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
                //TODO: make sure this volt makes it go INTO bumper
                motorArm.setVoltage(.6);
                break;

            case TRENCH_STORE:
                motorArm.setControl(request.withPosition(trenchStoreRevolutions));
                break;

            case STORE:
                    motorArm.setControl(request.withPosition(storeRevolutions));
                break;
            
            case INTAKING:
                //TODO: test this, never triggered due to being on store instead of intaking
                // if (hasReachedIntakeLocation && motorWheels.getMotorVoltage().getValueAsDouble() > 5 && motorWheels.getVelocity().getValueAsDouble() < 75) {
                //     motorArm.setControl(request.withPosition(logIntakeTarget.get() + 1.5));
                // } else {
                    motorArm.setControl(request.withPosition(logIntakeTarget.get()));
                // }
                break;
        }

        switch (currentState) {
            case IDLING:
                motorWheels.setControl(voltageRequest.withOutput(0));
                break;
            
            case INTAKING:
                // if (hasReachedIntakeLocation && motorWheels.getMotorVoltage().getValueAsDouble() > 5 && motorWheels.getVelocity().getValueAsDouble() < 75) {
                //     motorWheels.setControl(voltageRequest.withOutput(-logIntakingTarget.get()));
                // } else {
                    motorWheels.setControl(voltageRequest.withOutput(logIntakingTarget.get()));
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
    }

    private void updateReachedIntakeHeightLogic() {
        if (currentPos != IntakePos.INTAKING) {
            hasReachedIntakeLocation = false;
        } else {
            if (Math.abs(motorArm.getPosition().getValueAsDouble() - logIntakeTarget.get()) < 1) {
                hasReachedIntakeLocation = true;
            }
        }
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
    }

    public void zeroEncoders() {
        if (DriverStation.isDisabled()) {
            motorArm.setPosition(0);
        }
    }

    // ================== SYS ID COMMANDS ================
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}