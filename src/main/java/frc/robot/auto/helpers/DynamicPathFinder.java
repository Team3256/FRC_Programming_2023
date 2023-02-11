// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.Constants.DynamicPathGenerationConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * CREDIT FOR CODE: https://www.algorithms-and-technologies.com/a_star/java/ Adapted for Team 3256's
 * needs Used to perform the A-Star (A*) Algorithm to find the shortest path from a start to a
 * target node.
 */
public class DynamicPathFinder {
  /**
   * Finds the shortest distance between two nodes using the A-star algorithm
   *
   * @param graph an adjacency-matrix-representation of the graph where (x,y) is the weight of the
   *     edge or 0 if there is no edge. 
   * @param start the node to start from.
   * @param goal the node we're searching for
   * @return The shortest distance to the goal node. Can be easily modified to return the path.
   */
  public static ArrayList<Pose2D> findPath(double[][] graph, int start, int goal) {
    // This contains the distances from the start node to all other nodes
    double[] distances = new double[graph.length];
    Pose2D[] prev = new Pose2D[graph.length];

    // Initializing with a distance of "Infinity"
    Arrays.fill(distances, Double.MAX_VALUE);
    // The distance from the start node to itself is of course 0
    distances[start] = 0;

    // This contains the priorities with which to visit the nodes, calculated using
    // the heuristic.
    double[] priorities = new double[graph.length];
    // Initializing with a priority of "Infinity"
    Arrays.fill(priorities, Double.MAX_VALUE);
    // start node has a priority equal to straight line distance to goal. It will be
    // the first to be expanded.
    priorities[start] = heuristic(poseIndexes[start], poseIndexes[goal]);

    // This contains whether a node was already visited
    boolean[] visited = new boolean[graph.length];

    // While there are nodes left to visit...
    while (true) {

      // find the node with the currently lowest priority that has not been visited
      double lowestPriority = Integer.MAX_VALUE;
      int lowestPriorityIndex = -1;
      for (int i = 0; i < priorities.length; i++) {
        if (priorities[i] < lowestPriority && !visited[i]) {
          lowestPriority = priorities[i];
          lowestPriorityIndex = i;
        }
      }

      // no next node, no paths available return null
      if (lowestPriorityIndex == -1) {
        return null;
      } 
      // at goal node
      else if (lowestPriorityIndex == goal) {
        return distances[lowestPriorityIndex];
      }

      // System.out
      // .println("Visiting node " + lowestPriorityIndex + " with currently lowest
      // priority of " + lowestPriority);

      // ...then, for all neighboring nodes that haven't been visited yet....
      for (int i = 0; i < graph[lowestPriorityIndex].length; i++) {
        if (graph[lowestPriorityIndex][i] != 0 && !visited[i]) {
          // ...if the path over this edge is shorter...
          if (distances[lowestPriorityIndex] + graph[lowestPriorityIndex][i] < distances[i]) {
            // ...save this path as new shortest path
            distances[i] = distances[lowestPriorityIndex] + graph[lowestPriorityIndex][i];
            // ...and set the priority with which we should continue with this node
            priorities[i] = distances[i] + heuristic(poseIndexes[i], poseIndexes[goal]);
            System.out.println(
                "Updating distance of node "
                    + i
                    + " to "
                    + distances[i]
                    + " and priority to "
                    + priorities[i]);
          }
        }
      }

      // Lastly, note that we are finished with this node.
      visited[lowestPriorityIndex] = true;
      // System.out.println("Visited nodes: " + Arrays.toString(visited));
      // System.out.println("Currently lowest distances: " +
      // Arrays.toString(distances));

    }
  }

  /**
   * an estimation of distance from node x to y that is guaranteed to be lower than the actual distance
   * E.g. straight-line distance
   *  */  
  private static double heuristic(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  private static boolean isPathConnectionValid(Pose2d pose1, Pose2d pose2) {
    Translation2d translation1 = pose1.getTranslation();
    Translation2d translation2 = pose2.getTranslation();
    return true;
  }
}
