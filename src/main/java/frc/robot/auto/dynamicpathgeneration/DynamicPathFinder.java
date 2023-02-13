// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.FieldConstants;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.GeometryUtil;
import frc.robot.auto.dynamicpathgeneration.helpers.Path;
import frc.robot.auto.dynamicpathgeneration.helpers.Waypoint;
import frc.robot.swerve.SwerveConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class DynamicPathFinder {
  private int src;
  private Rotation2d srcRot;
  private int sink;
  private Rotation2d sinkRot;
  private int nodes;
  private ArrayList<Translation2d> positions;

  private double[] distanceToTravelToNodeN;
  private int[] previousNodesInCurrentPath;
  private double[] priorityQueue;

  static final double INF = Double.MAX_VALUE / 10;

  /**
   * Finds the fastest path between two nodes using Warrior-star algorithm
   *
   * @param src start node
   * @param sink end node
   */
  public DynamicPathFinder(
      int src,
      Rotation2d srcRot,
      int sink,
      Rotation2d sinkRot,
      ArrayList<Translation2d> positions) {
    this.src = src;
    this.srcRot = srcRot;
    this.sink = sink;
    this.sinkRot = sinkRot;
    this.positions = positions;
    this.nodes = positions.size();
    if (kDynamicPathGenerationDebug) {
      System.out.println("Running Path Finder Algorithm");
      System.out.println("src: " + src + ", sink: " + sink + ", nodes: " + nodes);
    }
  }

  public List<Translation2d> findPath() {
    // Time to travel from src to all other nodes
    distanceToTravelToNodeN = new double[nodes];
    Arrays.fill(distanceToTravelToNodeN, INF);
    distanceToTravelToNodeN[src] = 0;

    // Previous node in current optimal path to node
    previousNodesInCurrentPath = new int[nodes];

    // Priorities with which to visit the nodes
    priorityQueue = new double[nodes];
    Arrays.fill(priorityQueue, INF);
    priorityQueue[src] = heuristic(positions.get(src), positions.get(sink));

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
        // System.out.println("explore node:" + currentNode);
      }

      // No paths available
      if (currentNode == -1) {
        System.out.println("No paths available.");
        ArrayList<Integer> pathIds = new ArrayList<Integer>(Arrays.asList(nodes - 2, nodes - 1));
        return getPathPositionsFromPathIds(pathIds);
      }

      // Found shortest path to sink
      else if (currentNode == sink) {
        System.out.println("Path found");
        return getPathPositionsFromPathIds(getPathIdsInCurrentPath(sink));
      }

      // Update all unvisited neighboring nodes
      for (int node = 0; node < nodes; node++) {
        List<Integer> path = getPathIdsInCurrentPath(currentNode);
        if (positions.get(node).getX() < positions.get(currentNode).getX() && !visitedNodes[node]) {
          path.add(node);
          double pathTime = getPathTime(path);
          // If path over this edge is better
          if (pathTime < distanceToTravelToNodeN[node]) {
            // Save path as new current shortest path
            distanceToTravelToNodeN[node] = pathTime;
            previousNodesInCurrentPath[node] = currentNode;

            // Update node priority
            priorityQueue[node] =
                distanceToTravelToNodeN[node] + heuristic(positions.get(node), positions.get(sink));
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
  public static double heuristic(Translation2d position1, Translation2d position2) {
    double ret = position1.getDistance(position2) / SwerveConstants.kMaxSpeed;
    if (doesPathHitObstacles(position1, position2)) ret += INF / 2;
    return ret;
  }

  // calculate time to travel list of pathIds
  private double getPathTime(List<Integer> pathIds) {
    // make sure pathIds are valid (doesn't hit obstacles)
    for (int i = 0; i < pathIds.size() - 1; i++) {
      if (doesPathHitObstacles(positions.get(pathIds.get(i)), positions.get(pathIds.get(i + 1))))
        return INF;
    }

    // calc trajectory time
    PathPlannerTrajectory trajectory = getTrajectoryFromPathIds(pathIds);
    return trajectory.getTotalTimeSeconds();
  }

  // convert list of pathIds into PathPlannerTrajectory
  public PathPlannerTrajectory getTrajectoryFromPathIds(List<Integer> pathIds) {
    // convert pathIds into pathPoints
    List<Translation2d> pathPosition = getPathPositionsFromPathIds(pathIds);
    Path path = new Path(pathPosition, srcRot, sinkRot);
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }

    return PathPlanner.generatePath(dynamicPathConstraints, pathPoints);
  }

  // convert list of pathIds into list of pathPoses
  private List<Translation2d> getPathPositionsFromPathIds(List<Integer> pathIds) {
    List<Translation2d> pathPositions = new ArrayList<>();
    for (int node : pathIds) pathPositions.add(positions.get(node));
    return pathPositions;
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
  public static boolean doesPathHitObstacles(Translation2d position1, Translation2d position2) {
    for (Translation2d[] chargingStationCorner :
        FieldConstants.Community.kChargingStationSegments) {
      if (GeometryUtil.intersect(
          chargingStationCorner[0], chargingStationCorner[1], position1, position2)) {
        return true;
      }
    }
    return false;
  }
}
