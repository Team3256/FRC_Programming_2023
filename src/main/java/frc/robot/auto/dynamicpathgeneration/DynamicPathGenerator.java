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
import frc.robot.auto.dynamicpathgeneration.helpers.*;
import java.util.*;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final int src;
  private final Pose2d goalPose;
  private final int sink;
  private int numNodes;
  private ArrayList<PathNode> dynamicPathNodes;

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose) {
    System.out.println("Setting Up Path Finder Algorithm");
    this.startPose = startPose;
    this.goalPose = goalPose;
    dynamicPathNodes = new ArrayList<>(dynamicPathWayNodes);
    dynamicPathNodes.add(new PathNode(startPose.getTranslation()));
    dynamicPathNodes.add(new PathNode(goalPose.getTranslation()));
    numNodes = dynamicPathNodes.size();
    src = numNodes - 2;
    sink = numNodes - 1;

    connectToClosest(dynamicPathNodes.get(src), dynamicPathNodes);
    connectToClosest(dynamicPathNodes.get(sink), dynamicPathNodes);
  }

  public void connectToClosest(PathNode node, ArrayList<PathNode> nodes) {
    double closest = INF_TIME;
    PathNode ret = node;
    for (PathNode q : nodes) {
      if (q == node) continue;
      double dist = HeuristicHelper.splineHeuristic(node.getPoint(), q.getPoint());
      if (dist < closest) {
        closest = dist;
        ret = q;
      }
    }
    PathGenInit.fullyConnect(ret, node);
    System.out.println("closest to " + node + " is " + ret);
  }

  public List<Translation2d> getPositions() {
    DynamicPathFinder pathFinder = new DynamicPathFinder(src, sink, dynamicPathNodes);

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
