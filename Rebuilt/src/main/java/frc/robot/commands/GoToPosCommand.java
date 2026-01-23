package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * <p> Blue-Origin </p>
 * <p> Pose 2d
 * <ul>
 *   <li>X: Ending X</li>
 *   <li>Y: Ending Y</li>
 *   <li>Rotation: Ending rotation</li>
 * </ul>
 * </p>
 */
public class GoToPosCommand extends Command{

    Pose2d targetPose;

    PathConstraints constraints;

    Command pathfindingCommand;

    public GoToPosCommand(Pose2d targetPose) {
        this.targetPose = targetPose;

        constraints = new PathConstraints(
            0.8, 0.5,
            Units.degreesToRadians(90), Units.degreesToRadians(180) 
        );

        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose, 
            constraints,
            0.0
        );

        pathfindingCommand.initialize();
        pathfindingCommand.execute();
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(pathfindingCommand);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(pathfindingCommand);
    }

}
