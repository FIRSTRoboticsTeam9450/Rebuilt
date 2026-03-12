package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetIMU extends Command {
    
    CommandSwerveDrivetrain drivetrain;

    public ResetIMU(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.seedFieldCentric();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
