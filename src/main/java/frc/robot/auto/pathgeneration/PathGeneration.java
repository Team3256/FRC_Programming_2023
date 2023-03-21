// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration;

import static frc.robot.Constants.trajectoryViewer;
import static frc.robot.Constants.waypointViewer;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kDynamicPathGenerationDebug;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;

public class PathGeneration {
  public static Command createDynamicAbsolutePath(
      Pose2d start, Pose2d end, SwerveDrive swerveDrive, PathConstraints pathConstraints) {
    System.out.println("Running: Go to absolute " + end);

    // Small control lengths make the path a straight line
    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            pathConstraints,
            new PathPoint(start.getTranslation(), Rotation2d.fromDegrees(180), start.getRotation())
                .withControlLengths(0.01, 0.01),
            new PathPoint(end.getTranslation(), Rotation2d.fromDegrees(0), end.getRotation())
                .withControlLengths(0.01, 0.01));

    // send trajectory to networktables for logging
    if (kDynamicPathGenerationDebug) {
      trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(trajectory);
      waypointViewer.getObject("Src").setPose(trajectory.getInitialHolonomicPose());
      waypointViewer.getObject("Sink").setPose(trajectory.getEndState().poseMeters);
    }

    // create command that runs trajectory
    AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
    Command trajCommand = autoBuilder.createTrajectoryFollowCommand(trajectory, false, false);
    return trajCommand;
  }

  public static Command createDynamicRelativePath(
      SwerveDrive swerveDrive, Transform2d poseTransformation, PathConstraints pathConstraints) {
    return PathGeneration.createDynamicAbsolutePath(
        swerveDrive.getPose(),
        swerveDrive.getPose().transformBy(poseTransformation),
        swerveDrive,
        pathConstraints);
  }
}
