package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {
    enum State {
        RUN_IN,
        RUN_OUT,
        NO_RUN
    }
    private State state = State.NO_RUN;

    private static TransferSubsystem instance;

    // TODO: Motor IDs
    private final TalonFX lowerMotor = new TalonFX(-1, "CantDrive");
    private final TalonFX leftWallMotor = new TalonFX(-1, "CantDrive");
    private final TalonFX rightWallMotor = new TalonFX(-1, "CantDrive");
    private final TalonFX exitMotor = new TalonFX(-1, "CantDrive");
    
    private final TalonFXConfiguration config = new TalonFXConfiguration();
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
        lowerMotor.setVoltage(voltageSetpoint);
        leftWallMotor.setVoltage(voltageSetpoint);
        rightWallMotor.setVoltage(voltageSetpoint);
        exitMotor.setVoltage(voltageSetpoint);
    }

    private void configure() {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//InvertedValue.CounterClockwise_Positive;;
        lowerMotor.getConfigurator().apply(config);
        leftWallMotor.getConfigurator().apply(config);
        rightWallMotor.getConfigurator().apply(config);
        exitMotor.getConfigurator().apply(config);
    }

    public void setVoltageSetpoint(double volts) {
        voltageSetpoint = volts;
    }
    public double getVoltageSetpoint() {
        return voltageSetpoint;
    }
}
