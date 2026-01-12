package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;

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

    private TalonFX angleMotor = new TalonFX(ShooterConstants.MotorIDs.kAngleMotorID, RobotConstants.CANIVORE_BUS);
    private TalonFX shooterMotor = new TalonFX(ShooterConstants.MotorIDs.kShooterMotorID, RobotConstants.CANIVORE_BUS);


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
