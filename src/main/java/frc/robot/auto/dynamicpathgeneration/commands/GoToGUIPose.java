// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.swerve.SwerveDrive;

// TODO Extend PPTrajectoryFollowCommand
public class GoToGUIPose extends CommandBase {
  private final SwerveDrive swerveSubsystem;
  private final Pose2d goalPose;
  private AutoCommandRunner commandRunner;
  private PathPlannerTrajectory pathToScoringLocation;

  public GoToGUIPose(SwerveDrive swerveSubsystem, Pose2d goalPose) {
    this.swerveSubsystem = swerveSubsystem;
    this.goalPose = goalPose;

    addRequirements(swerveSubsystem);
  }

  public GoToGUIPose(SwerveDrive swerveSubsystem, Pose2d goalPose, AutoCommandRunner commandRunner) {
    this(swerveSubsystem, goalPose);

    // TODO Implement command runner
    this.commandRunner = commandRunner;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveSubsystem.getPose();
    DynamicPathGenerator pathGenerator = new DynamicPathGenerator(currentPose, goalPose);
    pathToScoringLocation = pathGenerator.getTrajectory();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // TODO FIXME
    return false;
  }
}
