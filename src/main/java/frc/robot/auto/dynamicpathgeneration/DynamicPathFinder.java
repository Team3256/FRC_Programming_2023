// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import frc.robot.swerve.SwerveConstants;
import java.util.*;

public class DynamicPathFinder {
  private final int src;
  private final int sink;
  private final int nodes;
  private final ArrayList<PathNode> pathNodes;

  private int[] pre;
  private double[] dist;

  public DynamicPathFinder(int src, int sink, ArrayList<PathNode> pathNodes) {
    this.src = src;
    this.sink = sink;
    this.pathNodes = pathNodes;
    this.nodes = pathNodes.size();
    if (kDynamicPathGenerationDebug) {
      System.out.println("Running Path Finder Algorithm");
      System.out.println("src: " + src + ", sink: " + sink + ", nodes: " + nodes);
    }
    // this.heuristic = HeuristicHelper.generateHeuristicTable(sink, positions);
  }

  public List<Integer> findPath() {
    int nodesExplored = 0;
    // Time to travel from src to all other nodes
    dist = new double[nodes];
    Arrays.fill(dist, INF_TIME);
    dist[src] = 0;

    // Previous node in current optimal path to node
    pre = new int[nodes];

    // Priorities with which to visit the nodes

    // Q to pop from
    PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> dist[a]));
    pq.add(src);

    // Visited nodes
    boolean[] visitedNodes = new boolean[nodes];

    while (!pq.isEmpty()) {
      // Find unvisited lowest priority node
      int currentNode = pq.poll();
      if (visitedNodes[currentNode]) continue;

      if (kDynamicPathGenerationDebug) {
        // System.out.println("explore node:" + currentNode);
      }
      nodesExplored++;

      // Found the shortest path to sink
      if (currentNode == sink) {
        System.out.println("Path found");
        return getPathIdsFromNode(sink);
      }

      // Update all unvisited neighboring nodes
      for (int childId = 0; childId < pathNodes.get(currentNode).getEdges().size(); childId++) {
        int next = pathNodes.get(currentNode).getEdges().get(childId).getIndex();
        if (visitedNodes[next]) continue;
        if (kDynamicPathGenerationDebug) {
          // System.out.println("explore child: " + next);
        }
        // calculate time
        double newDist =
            0.0001
                + dist[currentNode]
                + PathUtil.splineHeuristic(
                    pathNodes.get(currentNode).getPoint(), pathNodes.get(next).getPoint());
        // If path over this edge is better
        if (newDist < dist[next]) {
          // Save path as new current shortest path
          dist[next] = newDist;
          pre[next] = currentNode;

          // Add node to queue
          pq.add(next);
        }
      }

      // mark as visited
      visitedNodes[currentNode] = true;
    }
    // No paths available
    System.out.println("No paths available. Explored " + nodesExplored + " nodes.");

    ArrayList<Integer> pathIds = new ArrayList<Integer>(Arrays.asList(nodes - 2, nodes - 1));
    return pathIds;
  }

  // calculate time to travel list of pathIds
  private double getPathTime(List<Integer> pathIds) {
    // make sure pathIds are valid (doesn't hit obstacles)
    double totalDistance = 0;
    for (int i = 0; i < pathIds.size() - 1; i++) {
      if (PathUtil.doesPathSegmentHitObstacles(
          pathNodes.get(pathIds.get(i)).getPoint(), pathNodes.get(pathIds.get(i + 1)).getPoint()))
        return INF_TIME;
      totalDistance +=
          pathNodes
              .get(pathIds.get(i))
              .getPoint()
              .getDistance(pathNodes.get(pathIds.get(i + 1)).getPoint());
    }
    return totalDistance / SwerveConstants.kMaxSpeed;
  }

  // convert list of pathIds into list of pathPoses
  private List<Translation2d> getPositionsFromPathIds(List<Integer> pathIds) {
    List<Translation2d> pathPositions = new ArrayList<>();
    for (int node : pathIds) pathPositions.add(pathNodes.get(node).getPoint());
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

}
