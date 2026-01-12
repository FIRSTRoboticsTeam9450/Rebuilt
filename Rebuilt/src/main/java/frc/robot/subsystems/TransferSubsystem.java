package frc.robot.subsystems;

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
 * <p> To-Do:
 *  <p> - Motor IDs </p>
 *  <p> - Make Base Code </p>
 *  <p> - Get Actual Values </p>
 *  <p> - Add Logging </p>
 * </p>
 *  
 */
public class TransferSubsystem extends SubsystemBase {
    enum UsableState {
        RUN_IN,
        RUN_OUT,
        NO_RUN
    }
    private UsableState currentState = UsableState.NO_RUN;

    private static TransferSubsystem instance;

    private final TalonFX motor = new TalonFX(TransferConstants.MotorIDs.kDrivingMotorID, RobotConstants.CANIVORE_BUS);

    private VoltageOut voltageRequest = new VoltageOut(0);

    public TransferSubsystem() {
        configure();
    }
    public TransferSubsystem getInstance() {
        if (instance == null) instance = new TransferSubsystem();
        return instance;
    }

    @Override
    public void periodic() {
        applyState();
    }

    private void applyState() {
        switch (currentState) {
            case RUN_IN:
                voltageRequest.Output = 1;
                break;
            
            case RUN_OUT:
                voltageRequest.Output = -1;
                break;
            
            case NO_RUN:
                voltageRequest.Output = 0;
                break;
        }
        motor.setControl(voltageRequest);
    }

    private void configure() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        motor.getConfigurator().apply(motorConfig);
    }
}
