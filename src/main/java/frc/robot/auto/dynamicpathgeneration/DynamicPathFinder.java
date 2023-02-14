// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import frc.robot.swerve.SwerveConstants;
import java.util.*;

public class DynamicPathFinder {
  private final int src;
  private final Rotation2d srcRot;
  private final int sink;
  private final Rotation2d sinkRot;
  private final int nodes;
  private final ArrayList<Translation2d> positions;

  private int[] pre;
  private final double[] heuristic;

  /**
   * Finds the fastest path between two nodes using Warrior-star algorithm
   *
   * @param src start node
   * @param sink end node
   * @param heuristic heuristic
   */
  public DynamicPathFinder(
      int src,
      Rotation2d srcRot,
      int sink,
      Rotation2d sinkRot,
      ArrayList<Translation2d> positions,
      double[] heuristic) {
    this.src = src;
    this.srcRot = srcRot;
    this.sink = sink;
    this.sinkRot = sinkRot;
    this.positions = positions;
    this.nodes = positions.size();
    this.heuristic = heuristic;
    if (kDynamicPathGenerationDebug) {
      System.out.println("Running Path Finder Algorithm");
      System.out.println("src: " + src + ", sink: " + sink + ", nodes: " + nodes);
    }
  }

  public List<Translation2d> findPath() {
    int nodesExplored = 0;
    // Time to travel from src to all other nodes
    double[] distanceToTravelToNodeN = new double[nodes];
    Arrays.fill(distanceToTravelToNodeN, INF_TIME);
    distanceToTravelToNodeN[src] = 0;

    // Previous node in current optimal path to node
    pre = new int[nodes];

    // Priorities with which to visit the nodes
    double[] priority = new double[nodes];
    Arrays.fill(priority, INF_TIME);
    priority[src] = heuristic[src];

    // Q to pop from
    PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> priority[a]));
    pq.add(src);

    // Visited nodes
    boolean[] visitedNodes = new boolean[nodes];

    while (!pq.isEmpty()) {
      // Find unvisited lowest priority node
      int currentNode = pq.poll();
      if (visitedNodes[currentNode]) continue;

      if (kDynamicPathGenerationDebug) {
        // System.out.println("explore node:" + currentNode);
        nodesExplored++;
      }

      // Found shortest path to sink
      if (currentNode == sink) {
        System.out.println("Path found");
        return getPositionsFromPathIds(getPathIdsFromNode(sink));
      }

      // Update all unvisited neighboring nodes
      List<Integer> path = getPathIdsFromNode(currentNode);
      for (int node = 0; node < nodes; node++) {
        if (visitedNodes[node]) continue;
        if (positions.get(node).getX() > positions.get(currentNode).getX()) continue;
        // add node to path
        path.add(node);
        double pathTime = getPathTime(path);
        // If path over this edge is better
        if (pathTime < distanceToTravelToNodeN[node]) {
          // Save path as new current shortest path
          distanceToTravelToNodeN[node] = pathTime;
          pre[node] = currentNode;

          // Update node priority
          priority[node] = distanceToTravelToNodeN[node] + heuristic[node];

          // Add node to queue
          pq.add(node);
        }
        // pop node from path
        path.remove(path.size() - 1);
      }

      // mark as visited
      visitedNodes[currentNode] = true;
    }
    // No paths available
    System.out.println("No paths available. Explored " + nodesExplored + " nodes.");

    ArrayList<Integer> pathIds = new ArrayList<Integer>(Arrays.asList(nodes - 2, nodes - 1));
    return getPositionsFromPathIds(pathIds);
  }

  // calculate time to travel list of pathIds
  private double getPathTime(List<Integer> pathIds) {
    // make sure pathIds are valid (doesn't hit obstacles)
    double totalDistance = 0;
    for (int i = 0; i < pathIds.size() - 1; i++) {
      if (doesPathSegmentHitObstacles(
          positions.get(pathIds.get(i)), positions.get(pathIds.get(i + 1)))) return INF_TIME;
      totalDistance += positions.get(pathIds.get(i)).getDistance(positions.get(pathIds.get(i + 1)));
    }

    return totalDistance / SwerveConstants.kMaxSpeed;
  }

  // convert list of pathIds into list of pathPoses
  private List<Translation2d> getPositionsFromPathIds(List<Integer> pathIds) {
    List<Translation2d> pathPositions = new ArrayList<>();
    for (int node : pathIds) pathPositions.add(positions.get(node));
    return pathPositions;
  }

  // get the pathIds stored from src to node
  private List<Integer> getPathIdsFromNode(int node) {
    List<Integer> pathIds = new ArrayList<>();
    int currentNode = node;

    while (currentNode != src) {
      pathIds.add(currentNode);
      currentNode = pre[currentNode];
    }

    pathIds.add(currentNode);
    Collections.reverse(pathIds);
    return pathIds;
  }

  public static boolean doesLineHitObstacles(Translation2d position1, Translation2d position2) {
    for (Obstacle obstacle : obstacles) {
      if (obstacle.intersectsLineSegment(position1, position2)) return true;
    }
    return false;
  }

  public static boolean doesPathSegmentHitObstacles(
      Translation2d position1, Translation2d position2) {
    if (doesLineHitObstacles(position1, position2)) return true;

    Rotation2d normalAngle = position2.minus(position1).getAngle().plus(Rotation2d.fromDegrees(90));
    Translation2d normalVector = new Translation2d(1, 0).rotateBy(normalAngle).times(kRobotRadius);

    if (doesLineHitObstacles(position1.minus(normalVector), position2.minus(normalVector)))
      return true;
    if (doesLineHitObstacles(position1.plus(normalVector), position2.plus(normalVector)))
      return true;

    return false;
  }
}
