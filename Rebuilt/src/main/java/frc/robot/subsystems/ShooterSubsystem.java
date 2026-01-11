package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance;

    public enum AngleState {
        SHOOTING,
        FERRYING
    }

    public enum ShooterState {
        SHOOTING,
        NOT_SHOOTING
    }

    private AngleState currentAngleState;
    private ShooterState currentShooterState;

    private TalonFX angleMotor = new TalonFX(ShooterConstants.kAngleMotorID, "CantDrive");
    private TalonFX shooterMotor = new TalonFX(ShooterConstants.kShooterMotorID, "CantDrive");


    public ShooterSubsystem() {
        configureAngleMotors();
        configureShooterMotors();
    }

    public void configureAngleMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    }

    public void configureShooterMotors() {

    }

    @Override
    public void periodic() {
        applyState();
    }

    public void applyState() {
        switch (currentShooterState) {
            case SHOOTING:
                
                break;
        
            default:
                break;
        }
        switch (currentAngleState) {
            case SHOOTING:
                
                break;
            case FERRYING:

                break;
            default:
                break;
        }
    }

    /**
     * Set shooter state
     * @param wantedShooterState
     * @param wantedAngleState
     */
    public void setState(ShooterState wantedShooterState, AngleState wantedAngleState) {
        currentShooterState = wantedShooterState;
        currentAngleState = wantedAngleState;
    }

    public ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }
}
