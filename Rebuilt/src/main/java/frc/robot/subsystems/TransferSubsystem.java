package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TransferConstants;

/**
 * Subsystem which controls the handoff from Intake to Shooter.
 */
public class TransferSubsystem extends SubsystemBase {
    /** Transfer subsystem instance */
    private static volatile TransferSubsystem INSTANCE;

    /**
     * State machine for transfer operation modes
     */
    public enum TransferState {
        HOLD,
        RUN_IN,
        RUN_OUT,
        NO_RUN
    }
    private TransferState currentState = TransferState.NO_RUN;
    private TransferState previousState = currentState;

    private final TalonFX motor = new TalonFX(TransferConstants.DeviceIDs.kDrivingMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    /** Voltage request */
    private VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(RobotConstants.ENABLE_FOC);


    private boolean reverse = false;
    /**
     * Constructor - initializes the transfer subsystem and configures the motor
     */
    private TransferSubsystem() {
        configureMotor();
    }

    /**
     * Returns the single instance of the TransferSubsystem
     * @return The TurretSubsystem instance
     */
    public static synchronized TransferSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TransferSubsystem();
            INSTANCE.initialize();
        }
        return INSTANCE;
    }

    private void initialize() {
        Logger.recordOutput("Rebuilt/Transfer/Subsystem Initizalized", true);
    }

    /**
     * Configures the TalonFX motor
     */
    private void configureMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        motor.getConfigurator().apply(motorConfig);
    }

    /**
     * Periodic update function called every 20ms by the command scheduler
     */
    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    /**
     * State machine implementation - executes logic for current state
     */
    private void applyState() {
        // Set request voltage
        switch (currentState) {
            case HOLD: // Very slowly move fuel in
                if(reverse) {
                    voltageRequest.Output = -12;

                }
                voltageRequest.Output = 0.65;
                break;

            case RUN_IN: // Move fuel in
                if(reverse) {
                    voltageRequest.Output = -12;
                }
                voltageRequest.Output = 12;
                break;
            
            case RUN_OUT: // Move fuel out
                voltageRequest.Output = -12;
                break;
            
            case NO_RUN: // Don't move
                voltageRequest.Output = 0;
                break;
        }
        motor.setControl(voltageRequest);
    }

    /**
     * Changes the current state of the transfer
     * @param wantedState The desired state to transition to
     */
    public void setState(TransferState wantedState) {
        previousState = currentState;
        currentState = wantedState;
    }

    public Command setReverse(boolean value) {
        return new InstantCommand(() -> reverse = value);
    }

    /**
     * Get a command that sets the current state of the transfer
     * @param wantedState The desired state to transition to
     */
    public Command setStateCommand(TransferState wantedState) {
        return new InstantCommand(() -> setState(wantedState));
    }

    /**
     * Publishes telemetry data to AdvantageKit Logger
     */
    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Transfer/Current State", currentState);
        Logger.recordOutput("Rebuilt/Transfer/Previous State", previousState);

        Logger.recordOutput("Rebuilt/Transfer/Target Speed", voltageRequest.Output);
    }
}
