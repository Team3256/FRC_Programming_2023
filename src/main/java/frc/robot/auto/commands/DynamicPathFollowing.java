package frc.robot.auto.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.swerve.SwerveDrive;

public class DynamicPathFollowing extends CommandBase {
  private SwerveDrive swerveSubsystem;
  private Pose2d finalPose;
  private AutoCommandRunner commandRunner;
  private int[] obstacleGraph;

  static {

  }

  public DynamicPathFollowing(SwerveDrive swerveSubsystem, Pose2d finalPose) {
    this.swerveSubsystem = swerveSubsystem;
    this.finalPose = finalPose;

    addRequirements(swerveSubsystem);
  }

  public DynamicPathFollowing(SwerveDrive swerveSubsystem, Pose2d finalPose, AutoCommandRunner commandRunner) {
    this(swerveSubsystem, finalPose);
    this.commandRunner = commandRunner;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveSubsystem.getPose();
    PathPlannerTrajectory pathToNode = PathPlanner.generatePath(new PathConstraints(12, 8),
        currentPose, finalPose);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }
}
