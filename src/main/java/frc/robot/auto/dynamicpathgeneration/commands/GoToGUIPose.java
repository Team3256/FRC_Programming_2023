// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.auto.commands.PPTrajectoryFollowCommand;
import frc.robot.auto.dynamicpathgeneration.DynamicPathFollower;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.swerve.SwerveDrive;

// TODO Extend PPTrajectoryFollowCommand
public class GoToGUIPose extends PPTrajectoryFollowCommand {
  private final SwerveDrive swerveSubsystem;
  private final Pose2d goalPose;
  private AutoCommandRunner commandRunner;
  private PathPlannerTrajectory pathToScoringLocation;

  public GoToGUIPose() {
    super();
    SwerveDrive swerveSubsystem, AutoCommandRunner
  } commandRunner) {
    this.swerveSubsystem = swerveSubsystem;
    //TODO: GRAB FROM SD
    Pose2d goalPose = new Pose2d(0,0,new Rotation2d(0));
    this.commandRunner = commandRunner;
    this.goalPose = goalPose;
    addRequirements(swerveSubsystem);
    // TODO Implement command runner
    this.commandRunner = commandRunner;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveSubsystem.getPose();
    DynamicPathFollower pathGenerator = new DynamicPathFollower(currentPose, goalPose);
    pathToScoringLocation = pathGenerator.getTrajectory();
  }

  @Override
  public void execute() {
    autoCommand
  }

  @Override
  public boolean isFinished() {
    // TODO FIXME
    return false;
  }
}
