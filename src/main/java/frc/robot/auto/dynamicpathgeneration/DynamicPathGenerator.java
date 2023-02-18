// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.HeuristicHelper;
import frc.robot.auto.dynamicpathgeneration.helpers.Path;
import frc.robot.auto.dynamicpathgeneration.helpers.Waypoint;
import java.util.*;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final Pose2d goalPose;
  private int numNodes;
  private final double[] heuristic;
  private ArrayList<Translation2d> dynamicPathGraph;

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose) {
    this.startPose = startPose;
    this.goalPose = goalPose;
    init();
    this.heuristic = HeuristicHelper.generateHeuristicTable(numNodes - 1, dynamicPathGraph);
    if (kDynamicPathGenerationDebug) {
      // System.out.println("Heuristic Table:");
      // if (kDynamicPathGenerationDebug) {
      // for (double h : heuristic) {
      // System.out.println(h);
      // }
      // }
    }
  }

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose, double[] heuristic) {
    this.heuristic = heuristic;
    this.goalPose = goalPose;
    this.startPose = startPose;
    init();
  }

  public void init() {
    dynamicPathGraph = new ArrayList<>(dynamicPathAllowedPositions);
    dynamicPathGraph.add(startPose.getTranslation());
    dynamicPathGraph.add(goalPose.getTranslation());
    numNodes = dynamicPathGraph.size();
  }

  public List<Translation2d> getPositions() {
    DynamicPathFinder pathFinder =
        new DynamicPathFinder(
            numNodes - 2,
            startPose.getRotation(),
            numNodes - 1,
            goalPose.getRotation(),
            dynamicPathGraph,
            heuristic);

    List<Translation2d> positions = pathFinder.findPath();
    if (kDynamicPathGenerationDebug) {
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
