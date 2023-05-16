// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import java.util.*;

public class DynamicPathFinder {
  private final int src;
  private final int sink;
  private final int nodes;
  private final ArrayList<PathNode> pathNodes;

  private int[] pre;
  private double[] dist;

  /**
   * initialize dynamic path finder
   *
   * @param src index of src node in the graph
   * @param sink index of sink node in the graph
   * @param pathNodes graph to run algorithm on
   */
  public DynamicPathFinder(int src, int sink, ArrayList<PathNode> pathNodes) {
    this.src = src;
    this.sink = sink;
    this.pathNodes = pathNodes;
    this.nodes = pathNodes.size();
    System.out.println("Running Path Finder Algorithm");
    System.out.println("src: " + src + ", sink: " + sink + ", nodes: " + nodes);
  }

  /**
   * find shortest path between src and sink
   *
   * @return optimal path of nodes (indexes) to travel through to get from src to sink
   */
  public List<Integer> findPath() {
    int nodesExplored = 0;
    // Time to travel from src to all other nodes
    dist = new double[nodes];
    Arrays.fill(dist, INF_TIME);
    dist[src] = 0;

    // Previous node in current optimal path to node
    pre = new int[nodes];

    // queue to pop from, sorted based on lowest distance
    PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> dist[a]));
    pq.add(src);

    // Visited nodes
    boolean[] visitedNodes = new boolean[nodes];

    while (!pq.isEmpty()) {
      // Find unvisited lowest priority node
      int currentNode = pq.poll();
      if (visitedNodes[currentNode]) continue;
      nodesExplored++;

      // If the shortest path to sink has been found, return the path
      if (currentNode == sink) {
        System.out.println("Path found. Explored " + nodesExplored + " nodes.");
        List<Integer> pathIds = new ArrayList<>();
        while (currentNode != src) {
          pathIds.add(currentNode);
          currentNode = pre[currentNode];
        }
        pathIds.add(currentNode);
        Collections.reverse(pathIds);
        return pathIds;
      }

      // else update all unvisited neighboring nodes
      for (int childId = 0; childId < pathNodes.get(currentNode).getEdges().size(); childId++) {
        int next = pathNodes.get(currentNode).getEdges().get(childId);
        if (visitedNodes[next]) continue;
        // Calculate time (penalize paths with more points)
        double newDist =
            0.000001
                + dist[currentNode]
                + PathUtil.straightTravelTimeWithObstacles(
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
    // Sink is unreachable, return empty list
    System.out.println("No paths available. Explored " + nodesExplored + " nodes.");
    return new ArrayList<>();
  }
}
