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
import frc.robot.auto.dynamicpathgeneration.helpers.*;
import java.util.*;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final int src;
  private final Pose2d goalPose;
  private final int sink;
  private final int numNodes;
  private final ArrayList<PathNode> dynamicPathNodes;

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
    for (int i = 0; i < numNodes; i++) {
      dynamicPathNodes.get(i).setIndex(i);
    }
  }

  public PathNode connectToClosest(PathNode node, ArrayList<PathNode> nodes) {
    double closest = INF_TIME;
    PathNode ret = node;
    for (PathNode q : nodes) {
      if (q == node) continue;
      double dist = PathUtil.straightTravelTimeWithObstacles(node.getPoint(), q.getPoint());
      if (dist < closest) {
        closest = dist;
        ret = q;
      }
    }
    System.out.println("closest to " + node + " is " + ret);
    CreateDynamicPathWayNodes.fullyConnect(ret, node);
    return ret;
  }

  public List<Integer> getPathIds() {
    PathNode srcClosest = connectToClosest(dynamicPathNodes.get(src), dynamicPathNodes);
    PathNode sinkClosest = connectToClosest(dynamicPathNodes.get(sink), dynamicPathNodes);
    if (kDynamicPathGenerationDebug) {
      System.out.println("src edges:" + dynamicPathNodes.get(src).getEdges().size());
      System.out.println("sink edges:" + dynamicPathNodes.get(sink).getEdges().size());
    }
    DynamicPathFinder pathFinder = new DynamicPathFinder(src, sink, dynamicPathNodes);
    List<Integer> positions = pathFinder.findPath();
    if (kDynamicPathGenerationDebug) {
      System.out.println("This is the path generated:");
      System.out.println(positions);
    }
    CreateDynamicPathWayNodes.fullyDisconnect(srcClosest, dynamicPathNodes.get(src));
    CreateDynamicPathWayNodes.fullyDisconnect(sinkClosest, dynamicPathNodes.get(sink));
    return positions;
  }

  public List<PathNode> getPathNodes() {
    // convert pathIds into pathNodes
    List<Integer> pathIds = getPathIds();
    List<PathNode> pathNodes = new ArrayList<>();
    for (int i : pathIds) {
      pathNodes.add(dynamicPathNodes.get(i));
    }
    return pathNodes;
  }

  public PathPlannerTrajectory getTrajectory() {
    Path path = new Path(getPathNodes(), startPose.getRotation(), goalPose.getRotation());
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }
    // if no path points were found then there should be no trajectory
    if (pathPoints.size() == 0) return null;
    // convert pathPoints into Trajectory we return
    return PathPlanner.generatePath(dynamicPathConstraints, pathPoints);
  }
}
