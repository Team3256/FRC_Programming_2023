// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.Constants.FieldConstants;
import static frc.robot.auto.AutoConstants.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.helpers.GeometryUtil;
import frc.robot.helpers.Path;
import frc.robot.helpers.Waypoint;
import frc.robot.swerve.SwerveConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class DynamicPathFinder {
  private int src;
  private int sink;
  private int nodes;
  private ArrayList<Pose2d> poses;

  private double[] distanceToTravelToNodeN;
  private int[] previousNodesInCurrentPath;
  private double[] priorityQueue;

  static final double INF = Double.MAX_VALUE / 10;

  /**
   * Finds the fastest path between two nodes using Warrior-star algorithm
   *
   * @param src  start node
   * @param sink end node
   */
  public DynamicPathFinder(int src, int sink, ArrayList<Pose2d> poses) {
    this.src = src;
    this.sink = sink;
    this.poses = poses;
    this.nodes = poses.size();
  }

  public List<Pose2d> findPath() {
    // Time to travel from src to all other nodes
    distanceToTravelToNodeN = new double[nodes];
    Arrays.fill(distanceToTravelToNodeN, INF);
    distanceToTravelToNodeN[src] = 0;

    // Previous node in current optimal path to node
    previousNodesInCurrentPath = new int[nodes];

    // Priorities with which to visit the nodes
    priorityQueue = new double[nodes];
    Arrays.fill(priorityQueue, INF);
    priorityQueue[src] = heuristic(poses.get(src), poses.get(sink));

    // Visited nodes
    boolean[] visitedNodes = new boolean[nodes];

    while (true) {
      // Find unvisited lowest priority node
      double currentPriority = INF;
      int currentNode = -1;
      for (int node = 0; node < priorityQueue.length; node++) {
        if (priorityQueue[node] < currentPriority && !visitedNodes[node]) {
          currentPriority = priorityQueue[node];
          currentNode = node;
        }
      }
      if (kDynamicPathGenerationDebug) {
        System.out.println("cur:" + currentNode);
      }

      // No paths available
      if (currentNode == -1) {
        ArrayList<Integer> pathIds = new ArrayList<Integer>(Arrays.asList(nodes - 2, nodes - 1));
        return getPathPosesFromPathIds(pathIds);
      }

      // Found shortest path to sink
      else if (currentNode == sink) {
        if (kDynamicPathGenerationDebug) {
          System.out.println("Done!");
          System.out.println("Shortest Path Found:" + getPathIdsInCurrentPath(sink));
        }
        return getPathPosesFromPathIds(getPathIdsInCurrentPath(sink));
      }

      // Update all unvisited neighboring nodes
      for (int node = 0; node < nodes; node++) {
        List<Integer> path = getPathIdsInCurrentPath(currentNode);
        if (poses.get(node).getX() < poses.get(currentNode).getX() && !visitedNodes[node]) {
          path.add(node);
          double pathTime = getPathTime(path);
          // If path over this edge is better
          if (kDynamicPathGenerationDebug) {
            System.out.println("try:" + node);
            System.out.println("path:" + path);
            System.out.println("pathTime:" + pathTime);
          }
          if (pathTime < distanceToTravelToNodeN[node]) {
            // Save path as new current shortest path
            distanceToTravelToNodeN[node] = pathTime;
            previousNodesInCurrentPath[node] = currentNode;

            // Update node priority
            priorityQueue[node] = distanceToTravelToNodeN[node] + heuristic(poses.get(node), poses.get(sink));

            if (kDynamicPathGenerationDebug) {
              System.out.println("next:" + node);
              System.out.println("dist:" + distanceToTravelToNodeN[node]);
              System.out.println("heuristic:" + heuristic(poses.get(node), poses.get(sink)));
              System.out.println("priority:" + priorityQueue[node]);
            }
          }
          path.remove(path.size() - 1);
        }
      }

      // mark as visited
      visitedNodes[currentNode] = true;
    }
  }

  // heuristic estimate of time to travel 1->2 that is guaranteed to be lower than
  // actual
  public static double heuristic(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation()) / SwerveConstants.kMaxSpeed;
  }

  // make sure line segments don't intersect obstacles
  public static boolean isPathConnectionValid(Pose2d pose1, Pose2d pose2) {
    Translation2d translation1 = pose1.getTranslation();
    Translation2d translation2 = pose2.getTranslation();

    for (Translation2d[] chargingStationCorner : FieldConstants.Community.kChargingStationSegments) {
      if (GeometryUtil.intersect(
          chargingStationCorner[0], chargingStationCorner[1], translation1, translation2)) {
        return false;
      }
    }
    return true;
  }

  // calculate time to travel list of pathIds
  private double getPathTime(List<Integer> pathIds) {
    // make sure pathIds are valid (doesn't hit obstacles)
    for (int i = 0; i < pathIds.size() - 1; i++) {
      if (!doesPathHitObstacles(poses.get(pathIds.get(i)), poses.get(pathIds.get(i + 1))))
        return INF;
    }

    // calc trajectory time
    PathPlannerTrajectory trajectory = getTrajectoryFromPathIds(pathIds);
    return trajectory.getTotalTimeSeconds();
  }

  // convert list of pathIds into PathPlannerTrajectory
  public PathPlannerTrajectory getTrajectoryFromPathIds(List<Integer> pathIds) {
    // convert pathIds into pathPoints
    List<Pose2d> pathPoses = getPathPosesFromPathIds(pathIds);
    Path path = new Path(pathPoses);
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }

    return PathPlanner.generatePath(dynamicPathConstraints, pathPoints);
  }

  // convert list of pathIds into list of pathPoses
  private List<Pose2d> getPathPosesFromPathIds(List<Integer> pathIds) {
    List<Pose2d> pathPoses = new ArrayList<>();
    for (int node : pathIds)
      pathPoses.add(poses.get(node));
    return pathPoses;
  }
  // get the pathIds stored from src to node

  private List<Integer> getPathIdsInCurrentPath(int node) {
    List<Integer> pathIds = new ArrayList<>();
    int currentNode = node;

    while (currentNode != src) {
      pathIds.add(currentNode);
      currentNode = previousNodesInCurrentPath[currentNode];
    }

    pathIds.add(currentNode);
    Collections.reverse(pathIds);
    return pathIds;
  }

  // make sure line segments don't intersect obstacles
  public static boolean doesPathHitObstacles(Pose2d pose1, Pose2d pose2) {
    Translation2d translation1 = pose1.getTranslation();
    Translation2d translation2 = pose2.getTranslation();

    for (Translation2d[] chargingStationCorner : FieldConstants.Community.kChargingStationSegments) {
      if (GeometryUtil.intersect(
          chargingStationCorner[0], chargingStationCorner[1], translation1, translation2)) {
        System.out.println("Pose1:" + pose1 + ", Pose2:" + pose2 + " FAIL");
        return false;
      }
    }
    return true;
  }
}
