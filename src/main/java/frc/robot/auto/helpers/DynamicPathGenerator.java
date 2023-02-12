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
import frc.robot.helpers.Path;
import frc.robot.helpers.WayPoint;
import java.util.ArrayList;
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
    this.nodes = poseIndexes.length + 2;
  }

  public List<Pose2d> getPath() {
    ArrayList<Pose2d> poses = new ArrayList<>();
    Collections.addAll(poses, poseIndexes);
    poses.add(startPose);
    poses.add(goalPose);

    if (debug) {
      System.out.println("Path generated on poses:");
      System.out.println(poses);
    }

    DynamicPathFinder pathFinder = new DynamicPathFinder(nodes - 2, nodes - 1, poses);

    if (debug) {
      System.out.println("Path generated:");
      System.out.println(pathFinder.findPath());
    }
    return pathFinder.findPath();
  }

  public PathPlannerTrajectory getTrajectory() {
    // convert pathPoses into pathPoints
    List<Pose2d> pathPoses = getPath();
    Path path = new Path(pathPoses);
    List<PathPoint> pathPoints = new ArrayList<>();
    for (WayPoint way : path.getWaypoints()) {
      pathPoints.add(way.toPathPoint());
    }

    // convert pathPoints into Trajectory we return
    return PathPlanner.generatePath(dynamicPathConstraints, pathPoints);
  }
}
