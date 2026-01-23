package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeSubsystem extends SubsystemBase{

    private static IntakeSubsystem INSTACNE;

    //List the states the intake will use
    public enum IntakePos {
        ZEROING,
        STORE,
        INTAKING,
    }

    public enum IntakeState {
        IDLING,
        INTAKING,
    }

    //Set the current state of the robot to be idle
    private IntakePos currentPos = IntakePos.STORE;
    private IntakeState currentState = IntakeState.IDLING;
    
    //Set up motors
    TalonFX motorArm = new TalonFX(IntakeConstants.DeviceIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    TalonFX motorWheels = new TalonFX(IntakeConstants.DeviceIDs.kWheelMotorID, RobotConstants.CANIVORE_BUS);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Intake/Angle/Velocity", 45);
    private LoggedNetworkNumber logMMAccel = new LoggedNetworkNumber("/Tuning/Intake/Angle/Acceleration", 30);
    private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Intake/Angle/Jerk", 1000);

    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Intake/Angle/kS", 0.1);
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Intake/Angle/kV", 0.33);
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Intake/Angle/kA", 0);
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Intake/Angle/kP", 25);
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Intake/Angle/kI", 0);
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Intake/Angle/kD", 0);
    private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Intake/Angle/kG", 0);

    private LoggedNetworkNumber logIntakeTarget = new LoggedNetworkNumber("/Tuning/Intake/Angle/Intaking Target", 7.39209);
    private LoggedNetworkNumber logIntakingTarget = new LoggedNetworkNumber("/Tuning/Intake/Wheels/Target", 10);

    //Motion Magic Set Up(Intake arm)
    private MotionMagicVoltage request = new MotionMagicVoltage(0);

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotionMagic.MotionMagicAcceleration = logMMAccel.get();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = logMMVeloc.get();
        motorConfig.MotionMagic.MotionMagicJerk = logMMJerk.get();

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

    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    private void applyState() {
        switch (currentPos) {
            case ZEROING:
                break;

            case STORE:
                motorArm.setControl(request.withPosition(0.5));
                break;
            
            case INTAKING:
                motorArm.setControl(request.withPosition(logIntakeTarget.get()));
                break;
        }

        switch (currentState) {
            case IDLING:
                motorWheels.setVoltage(0);
                break;
            
            case INTAKING:
                motorWheels.setVoltage(logIntakingTarget.get());
                break;
        }
    }

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Intake/Angle/Current Pos", currentPos);

        Logger.recordOutput("Rebuilt/Intake/Angle/Position", motorArm.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Target", currentPos == IntakePos.INTAKING ? logIntakeTarget.get() : 0.5);
        Logger.recordOutput("Rebuilt/Intake/Angle/Motor Voltage", motorArm.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Motor Velocity", motorArm.getVelocity().getValueAsDouble());
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

    public static IntakeSubsystem getInstance() {
        if (INSTACNE == null) INSTACNE = new IntakeSubsystem();
        return INSTACNE;
    }

    public void updateConstants() {
        boolean gainsChanged = false;

        if (motorConfig.MotionMagic.MotionMagicAcceleration != logMMAccel.get() ||
            motorConfig.MotionMagic.MotionMagicCruiseVelocity != logMMVeloc.get() ||
            motorConfig.MotionMagic.MotionMagicJerk != logMMJerk.get() ||
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
        motorArm.setPosition(0);
    }
}