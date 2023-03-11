// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.simplepathgeneration;

import static frc.robot.Constants.trajectoryViewer;
import static frc.robot.Constants.waypointViewer;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kDynamicPathConstraints;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kDynamicPathGenerationDebug;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;

public class SimpleGoToAbsolute {
  static Command run(SwerveDrive swerveDrive, Pose2d sink) {
    Pose2d src = swerveDrive.getPose();
    Rotation2d heading = sink.minus(src).getTranslation().getAngle();
    PathPlannerTrajectory traj =
        PathPlanner.generatePath(
            kDynamicPathConstraints,
            new PathPoint(src.getTranslation(), heading, src.getRotation()),
            new PathPoint(sink.getTranslation(), heading, sink.getRotation()));

    // send trajectory to networktables for logging
    if (kDynamicPathGenerationDebug) {
      trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(traj);
      waypointViewer.getObject("Src").setPose(traj.getInitialHolonomicPose());
      waypointViewer.getObject("Sink").setPose(traj.getEndState().poseMeters);
    }

    // create command that runs trajectory
    AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
    Command trajCommand = autoBuilder.createPathPlannerCommand(traj, false, false);
    return trajCommand;
  }
}
