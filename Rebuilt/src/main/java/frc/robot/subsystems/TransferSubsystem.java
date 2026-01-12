package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TransferConstants;

/**
 * Subsystem which controls the handoff from Intake to Shooter.
 * 
 * <p> To-Do:
 *  <p> - Make Base Code </p>
 *  <p> - Get Actual Values </p>
 *  <p> - Add Logging </p>
 * </p>
 *  
 */
public class TransferSubsystem extends SubsystemBase {
    enum State {
        RUN_IN,
        RUN_OUT,
        NO_RUN
    }
    private State state = State.NO_RUN;

    private static TransferSubsystem instance;

    // TODO: Motor IDs
    private final TalonFX motor = new TalonFX(TransferConstants.MotorIDs.drivingMotor, RobotConstants.CANIVORE_BUS);

    private double voltageSetpoint = 0.0;

    public TransferSubsystem() {
        configure();
    }
    public TransferSubsystem getInstance() {
        if (instance == null) instance = new TransferSubsystem();
        return instance;
    }

    @Override
    public void periodic() {

    }

    private void configure() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        motor.getConfigurator().apply(motorConfig);
    }

    public void setVoltageSetpoint(double volts) {
        voltageSetpoint = volts;
    }
    public double getVoltageSetpoint() {
        return voltageSetpoint;
    }
}
