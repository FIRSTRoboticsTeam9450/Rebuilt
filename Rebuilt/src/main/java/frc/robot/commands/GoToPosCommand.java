package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

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
        addRequirements(RobotContainer.drivetrain);
        this.targetPose = targetPose;

        constraints = new PathConstraints(
            1, 1,
            Units.degreesToRadians(30), Units.degreesToRadians(60) 
        );
    }

    @Override
    public void initialize() {
        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose, 
            constraints,
            0.0
        );
        
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
