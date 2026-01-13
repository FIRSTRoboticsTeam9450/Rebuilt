package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private static ClimberSubsystem climber;

    private double voltage;
    private double maxVoltage;

    private PIDController climbPIDController = new PIDController(0, 0, 0);

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem(){}

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
