// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.Path;
import frc.robot.auto.dynamicpathgeneration.helpers.Waypoint;
import java.util.ArrayList;
import java.util.List;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final Pose2d goalPose;
  private final int nodes;
  private static final boolean debug = true;

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose) {
    this.startPose = startPose;
    this.goalPose = goalPose;
    this.nodes = dynamicPathPositions.size() + 2;
    for (double x = xi; x < xf; x += dx) {
      for (double y = yi; y < yf; y += dy) {
        dynamicPathPositions.add(new Translation2d(x, y));
      }
    }
  }

  public List<Translation2d> getPath() {
    ArrayList<Translation2d> positions = new ArrayList<>(dynamicPathPositions);
    positions.add(startPose.getTranslation());
    positions.add(goalPose.getTranslation());

    if (debug) {
      System.out.println("Path generated on poses:");
      System.out.println(positions);
    }

    DynamicPathFinder pathFinder =
        new DynamicPathFinder(
            nodes - 2, startPose.getRotation(), nodes - 1, goalPose.getRotation(), positions);

    if (debug) {
      System.out.println("Path generated:");
      System.out.println(pathFinder.findPath());
    }
    return pathFinder.findPath();
  }

  public PathPlannerTrajectory getTrajectory() {
    // convert pathPoses into pathPoints
    List<Translation2d> pathPoses = getPath();
    Path path = new Path(pathPoses, startPose.getRotation(), goalPose.getRotation());
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }

    // convert pathPoints into Trajectory we return
    return PathPlanner.generatePath(dynamicPathConstraints, pathPoints);
  }
}
