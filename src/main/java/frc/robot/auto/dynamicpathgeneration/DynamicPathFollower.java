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
import static frc.robot.led.LEDConstants.*;

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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.led.LED;
import frc.robot.led.commands.SetAllBlink;
import frc.robot.swerve.SwerveDrive;
import java.util.function.Supplier;

public class DynamicPathFollower {
  public enum GoalType {
    LOW_GRID,
    MID_GRID,
    HIGH_GRID,
    DOUBLE_STATION_TOP,
    DOUBLE_STATION_BOTTOM;
  }

  public static Command run(SwerveDrive swerveDrive, GoalType goalType, LED ledSubsystem) {
    return run(swerveDrive, goalType, ledSubsystem, false, () -> new InstantCommand());
  }

  public static Command run(
      SwerveDrive swerveDrive,
      GoalType goalType,
      LED ledSubsystem,
      boolean useInBetweenCommand,
      Supplier<Command> inBetweenTrajectoryCommand) {
    long ms0 = System.currentTimeMillis();
    // get src, sink
    Pose2d src = swerveDrive.getPose();
    Pose2d finalSink = new Pose2d();
    Pose2d dynamicPathGenSink = new Pose2d();

    if (goalType == GoalType.DOUBLE_STATION_BOTTOM) {
      dynamicPathGenSink = kBlueBottomDoubleSubstationPose.transformBy(kSubstationPreSink);
      finalSink = kBlueBottomDoubleSubstationPose;
    } else if (goalType == GoalType.DOUBLE_STATION_TOP) {
      dynamicPathGenSink = kBlueTopDoubleSubstationPose.transformBy(kSubstationPreSink);
      finalSink = kBlueTopDoubleSubstationPose;
    }

    if (goalType != GoalType.DOUBLE_STATION_TOP && goalType != GoalType.DOUBLE_STATION_BOTTOM) {
      int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
      if (locationId == -1) {
        System.out.println("locationId was invalid");
        if (ledSubsystem != null) {
          new SetAllBlink(ledSubsystem, kError);
        } else {
          return new InstantCommand();
        }
      }
      if (goalType == GoalType.LOW_GRID) {
        finalSink = kBottomBlueScoringPoses[locationId];
      } else if (goalType == GoalType.MID_GRID) {
        finalSink = kMidBlueScoringPoses[locationId];
      } else if (goalType == GoalType.HIGH_GRID) {
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
    DynamicPathGenerator generator = new DynamicPathGenerator(src, dynamicPathGenSink, swerveDrive);
    PathPlannerTrajectory dynamicPathGenTrajectory = generator.getTrajectory();

    // handle invalid trajectory
    if (dynamicPathGenTrajectory == null) {
      System.out.println("No trajectory was found.");
      return new SetAllBlink(ledSubsystem, kError);
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

    Command scoringLocationTrajectoryCommand;
    if (goalType != GoalType.HIGH_GRID) {
      PathPoint dynamicPathGenEnd =
          new PathPoint(
              dynamicPathGenSink.getTranslation(),
              new Rotation2d(),
              dynamicPathGenSink.getRotation());
      PathPoint scoringLocation =
          new PathPoint(finalSink.getTranslation(), new Rotation2d(), finalSink.getRotation(), 2);
      PathPlannerTrajectory trajectoryToFinalSink =
          PathPlanner.generatePath(
              kPathToDestinationConstraints, dynamicPathGenEnd, scoringLocation);
      scoringLocationTrajectoryCommand =
          autoBuilder.createPathPlannerCommand(trajectoryToFinalSink, false, false);
    } else {
      scoringLocationTrajectoryCommand = new InstantCommand();
    }

    // run command
    System.out.println("Time to run command: " + (System.currentTimeMillis() - ms0));

    Command finalTrajectory = scoringLocationTrajectoryCommand;
    if (useInBetweenCommand) {
      finalTrajectory =
          new ParallelDeadlineGroup(
              new WaitCommand(1.5).andThen(scoringLocationTrajectoryCommand),
              inBetweenTrajectoryCommand.get().asProxy());
    }

    return Commands.sequence(
        dynamicPathGenTrajectoryCommand, finalTrajectory, new SetAllBlink(ledSubsystem, kSuccess));
  }
}
