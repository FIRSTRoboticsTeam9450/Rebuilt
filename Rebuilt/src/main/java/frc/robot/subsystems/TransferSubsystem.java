package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TransferConstants;

/**
 * Subsystem which controls the handoff from Intake to Shooter.
 * 
 * <p>
 *  <p> TODO: Get actual values (Motor ID, direction, and voltages) </p>
 * </p>
 */
public class TransferSubsystem extends SubsystemBase {
    /** Transfer subsystem instance */
    private static TransferSubsystem INSTANCE;

    /**
     * State machine for transfer operation modes
     */
    public enum TransferState {
        RUN_IN,
        RUN_OUT,
        NO_RUN
    }
    private TransferState currentState = TransferState.NO_RUN;
    private TransferState previousState = currentState;

    private final TalonFX motor = new TalonFX(TransferConstants.DeviceIDs.kDrivingMotorID, RobotConstants.CANIVORE_BUS);

    /** Voltage request */
    private VoltageOut voltageRequest = new VoltageOut(0.0);

    /**
     * Constructor - initializes the transfer subsystem and configures the motor
     */
    private TransferSubsystem() {
        configure();
    }

    /**
     * Returns the single instance of the TransferSubsystem
     * @return The TurretSubsystem instance
     */
    public static TransferSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new TransferSubsystem();
        return INSTANCE;
    }

    /**
     * Configures the TalonFX motor
     */
    private void configure() {
        // Create config
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Set motor direction
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Apply config
        motor.getConfigurator().apply(motorConfig);
    }

    /**
     * Periodic update function called every 20ms by the command scheduler
     */
    @Override
    public void periodic() {
        // Execute current state logic
        applyState();

        // Log telemetry data
        publishLogs();
    }

    /**
     * State machine implementation - executes logic for current state
     */
    private void applyState() {
        // Set request voltage
        switch (currentState) {
            case RUN_IN: // Move fuel in
                voltageRequest.Output = 8;
                break;
            
            case RUN_OUT: // Move fuel out
                voltageRequest.Output = -8;
                break;
            
            case NO_RUN: // Don't move
                voltageRequest.Output = 0;
                break;
        }

        // Apply voltage to motor
        motor.setControl(voltageRequest);
    }

    /**
     * Changes the current state of the transfer
     * @param wantedState The desired state to transition to
     */
    public void setState(TransferState wantedState) {
        // Set states
        previousState = currentState;
        currentState = wantedState;
    }

    /**
     * Publishes telemetry data to AdvantageKit Logger
     */
    private void publishLogs() {
        // Log state data
        Logger.recordOutput("Rebuilt/Transfer/Current State", currentState);
        Logger.recordOutput("Rebuilt/Transfer/Previous State", previousState);

        // Log motor data
        Logger.recordOutput("Rebuilt/Transfer/Target Speed", voltageRequest.Output);
    }
}
