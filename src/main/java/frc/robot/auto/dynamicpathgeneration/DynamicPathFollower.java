// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.trajectoryViewer;
import static frc.robot.Constants.waypointViewer;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;

public class DynamicPathFollower {
  public enum GoalType {
    LOW_GRID,
    MID_GRID,
    HIGH_GRID,
    STATION;
  }

  public static void run(SwerveDrive swerveDrive, GoalType type) {
    long ms0 = System.currentTimeMillis();
    // get src, sink
    Pose2d src = swerveDrive.getPose();
    Pose2d sink = kBlueStationPose;
    if (type != GoalType.STATION) {
      // TODO: change random to -1 when not testing
      int rand = (int) (Math.random() * 9);
      int locationId = (int) SmartDashboard.getNumber("locationId", rand);
      // handle invalid location
      if (locationId == -1) {
        System.out.println("LocationId entered was invalid.");
        return;
      }
      if (type == GoalType.LOW_GRID) {
        sink = kBottomBlueScoringPoses[locationId];
      } else if (type == GoalType.MID_GRID) {
        sink = kMidBlueScoringPoses[locationId];
      } else if (type == GoalType.HIGH_GRID) {
        sink = kHighBlueScoringPoses[locationId];
      }
    }
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      sink = PathUtil.flip(sink);
    }
    // get trajectory
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    PathPlannerTrajectory trajectory = generator.getTrajectory();
    // handle invalid trajectory
    if (trajectory == null) {
      System.out.println("No trajectory was found.");
      return;
    } else {
      System.out.println("Trajectory was found.");
    }

    System.out.println("Time to find trajectory: " + (System.currentTimeMillis() - ms0));
    // send trajectory to networktables for logging
    if (kDynamicPathGenerationDebug) {
      trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(trajectory);
      waypointViewer.getObject("Src").setPose(trajectory.getInitialHolonomicPose());
      waypointViewer.getObject("Sink").setPose(trajectory.sample(1000).poseMeters);
    }
    // create command that runs trajectory
    AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
    Command pathPlannerCommand = autoBuilder.createPathPlannerCommand(trajectory, false, false);
    // run command
    System.out.println("Time to run command: " + (System.currentTimeMillis() - ms0));
    System.out.println();
    pathPlannerCommand.schedule();
  }
}
