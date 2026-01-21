package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * TODO: Vision needs to be fixed before this can be used
 */
public class GoToPosCommand extends Command{

    Pose2d targetPose;

    PathConstraints constraints;

    public GoToPosCommand(Pose2d targetPose) {
        this.targetPose = targetPose;

        constraints = new PathConstraints(
            1, 1,
            Units.degreesToRadians(540), Units.degreesToRadians(720) 
            );
    }

    @Override
    public void initialize() {
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose, 
            constraints
            );
        
        CommandScheduler.getInstance().schedule(pathfindingCommand);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
