package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveBrakeCommand extends Command{

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveDriveBrake();
    
    public DriveBrakeCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drivetrain.setControl(brakeRequest);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveRequest.FieldCentric stopRequest = new FieldCentric();
        drivetrain.setControl(stopRequest);
    }

}
