package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

    private static ClimberSubsystem climber;

    private double voltage;
    private double maxVoltage;

    private TalonFX climbMotor = new TalonFX(0, RobotConstants.CANIVORE_BUS_NAME);

    private CANcoder climbEncoder = new CANcoder(0, RobotConstants.CANIVORE_BUS_NAME);

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem() {
        TalonFXConfigurator climbConfig = climbMotor.getConfigurator();
    }

    /* ----------- Updaters ----------- */
    public void setVoltage(double voltage) {}

    public void setMaxVoltage(double maxVoltage) {}

    public void updatePIDs() {}

    public void resetEncoders() {}

    public void periodic() {}

    public void getPosition() {} //change return type to "double" after encoders are instantiated

    public static ClimberSubsystem getInstance() {
        if (climber == null) {
            climber = new ClimberSubsystem();
        }

        return climber;
    }
    
}
