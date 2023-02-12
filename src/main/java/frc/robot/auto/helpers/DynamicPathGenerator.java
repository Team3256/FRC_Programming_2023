// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.auto.AutoConstants.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final Pose2d goalPose;
  private final int nodes;
  private static final boolean debug = true;

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose) {
    this.startPose = startPose;
    this.goalPose = goalPose;
    this.nodes =  poseIndexes.length+2;
  }

  public List<Pose2d> computePath() {
    ArrayList<Pose2d> poses = new ArrayList<>();
    Collections.addAll(poses,poseIndexes);
    poses.add(startPose);
    poses.add(goalPose);

    if (debug) {
      System.out.println("Poses:");
      System.out.println(poses);
    }

    DynamicPathFinder pathFinder =
        new DynamicPathFinder(  nodes- 2, nodes- 1, poses);

    List<Pose2d> ret = new ArrayList<>();
    ArrayList<Integer> pathIndexes = pathFinder.findPath();
    for (int index : pathIndexes) {
      ret.add(poses.get(index));
    }
    if (debug) {
      System.out.println("pathIndexes:");
      System.out.println(pathIndexes);
    }
    return ret;
  }

  public PathPlannerTrajectory computeTrajectory() {
    List<Pose2d> path = computePath();
    List<PathPoint> waypoints = new ArrayList<>();
    waypoints.add(
        new PathPoint(startPose.getTranslation(), new Rotation2d(), startPose.getRotation()));
    for (Pose2d pointPose : path) {
      waypoints.add(new PathPoint(pointPose.getTranslation(), pointPose.getRotation()));
    }
    waypoints.add(
        new PathPoint(goalPose.getTranslation(), new Rotation2d(), goalPose.getRotation()));
    return PathPlanner.generatePath(dynamicPathConstraints, waypoints);
  }
}
