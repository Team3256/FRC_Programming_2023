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

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public static Command run(SwerveDrive swerveDrive, GoalType type) {
    long ms0 = System.currentTimeMillis();
    // get src, sink
    Pose2d src = swerveDrive.getPose();
    Pose2d finalSink = kBlueStationPose;
    Pose2d dynamicPathGenSink = kBlueStationPose;
    if (type != GoalType.STATION) {
      int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
      System.out.println("Location id: " + locationId);
      // handle invalid location
      if (locationId == -1) {
        System.out.println("LocationId entered was invalid.");
        return new InstantCommand();
        // Show error LEDs
        // return new LEDSetAllSectionsPattern(ledSubsystem, ledPattern);
      }
      if (type == GoalType.LOW_GRID) {
        finalSink = kBottomBlueScoringPoses[locationId];
      } else if (type == GoalType.MID_GRID) {
        finalSink = kMidBlueScoringPoses[locationId];
      } else if (type == GoalType.HIGH_GRID) {
        finalSink = kHighBlueScoringPoses[locationId];
      }
      dynamicPathGenSink = kHighBlueScoringPoses[locationId];
    }
    System.out.println("Sink: " + finalSink);
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      finalSink = PathUtil.flip(finalSink);
      dynamicPathGenSink = PathUtil.flip(dynamicPathGenSink);
    }
    // get trajectory
    DynamicPathGenerator generator = new DynamicPathGenerator(src, dynamicPathGenSink);
    PathPlannerTrajectory dynamicPathGenTrajectory = generator.getTrajectory();

    PathPoint dynamicPathGenEnd =
        new PathPoint(
            dynamicPathGenSink.getTranslation(),
            new Rotation2d(),
            dynamicPathGenSink.getRotation());
    PathPoint scoringLocation =
        new PathPoint(finalSink.getTranslation(), new Rotation2d(), finalSink.getRotation(), 2);
    PathPlannerTrajectory trajectoryToFinalSink =
        PathPlanner.generatePath(kPathToScoreConstraints, dynamicPathGenEnd, scoringLocation);
    // handle invalid trajectory
    if (dynamicPathGenTrajectory == null) {
      System.out.println("No trajectory was found.");
      return new InstantCommand();
    } else {
      System.out.println("Trajectory was found.");
    }

    System.out.println("Time to find trajectory: " + (System.currentTimeMillis() - ms0));
    // send trajectory to networktables for logging
    if (kDynamicPathGenerationDebug) {
      trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(dynamicPathGenTrajectory);
      waypointViewer.getObject("Src").setPose(dynamicPathGenTrajectory.getInitialHolonomicPose());
      waypointViewer.getObject("Sink").setPose(dynamicPathGenTrajectory.getEndState().poseMeters);
    }
    // create command that runs trajectory
    AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
    Command dynamicPathGenTrajectoryCommand =
        autoBuilder.createPathPlannerCommand(dynamicPathGenTrajectory, false, false);
    Command scoringLocationTrajectoryCommand =
        autoBuilder.createPathPlannerCommand(trajectoryToFinalSink, false, false);
    // run command
    System.out.println("Time to run command: " + (System.currentTimeMillis() - ms0));

    return Commands.sequence(dynamicPathGenTrajectoryCommand, scoringLocationTrajectoryCommand);
  }
}
