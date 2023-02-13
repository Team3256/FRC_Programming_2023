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
    double r = 0.45;
    int res = 100;
    if (dynamicPathAllowedPositions.size() == 0) {
      for (double x=searchLowX;x <searchHiX;x +=searchResX) {
        for (double y=searchLowY;y <searchHiY;y +=searchResY) {
          boolean bad = false;
          for (int i = 0; i < res; i++) {
            double angle = i * 2 * Math.PI / res;
            if (chargingStation.containsPoint(
                new Translation2d(x + r * Math.cos(angle), y + r * Math.sin(angle)))) bad = true;
          }
          if (!bad) dynamicPathAllowedPositions.add(new Translation2d(x, y));
        }
      }
    }
    this.nodes = dynamicPathAllowedPositions.size() + 2;
  }

  public List<Translation2d> getPositions() {
    ArrayList<Translation2d> newDynamicPathAllowedPositions =
        new ArrayList<>(dynamicPathAllowedPositions);
    newDynamicPathAllowedPositions.add(startPose.getTranslation());
    newDynamicPathAllowedPositions.add(goalPose.getTranslation());

    DynamicPathFinder pathFinder =
        new DynamicPathFinder(
            nodes - 2,
            startPose.getRotation(),
            nodes - 1,
            goalPose.getRotation(),
            newDynamicPathAllowedPositions);

    List<Translation2d> positions = pathFinder.findPath();
    if (debug) {
      System.out.println("This is the path generated:");
      System.out.println(positions);
    }
    return positions;
  }

  public PathPlannerTrajectory getTrajectory() {
    // convert pathPoses into pathPoints
    List<Translation2d> pathPoses = getPositions();
    Path path = new Path(pathPoses, startPose.getRotation(), goalPose.getRotation());
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }

    // convert pathPoints into Trajectory we return
    return PathPlanner.generatePath(dynamicPathConstraints, pathPoints);
  }
}
