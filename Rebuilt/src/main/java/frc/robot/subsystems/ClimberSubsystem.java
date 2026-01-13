package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    public ClimberSubsystem climber;

    /* ----------- Initializaton ----------- */
    public ClimberSubsystem(){}

    /* ----------- Updaters ----------- */
    public void setVoltage(double voltage) {}

    public void setMaxVoltage(double maxVoltage) {}

    public void updatePIDs() {}

    public void resetEncoders() {}

    public void periodic() {}

    public ClimberSubsystem getInstance() {
        if (climber == null) {
            climber = new ClimberSubsystem();
        }

        return climber;
    }
    
}
