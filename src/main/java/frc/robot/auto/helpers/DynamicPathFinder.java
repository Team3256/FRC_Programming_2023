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
  public static ArrayList<Integer> findPath(double[][] graph, int start, int goal) {
    // This contains the distances from the start node to all other nodes
    double[] distances = new double[graph.length];
    int[] prev = new int[graph.length];

    // Initializing with a distance of "Infinity"
    Arrays.fill(distances, Double.MAX_VALUE);
    // The distance from the start node to itself is of course 0
    distances[start] = 0;

    // This contains the priorities with which to visit the nodes, calculated using the heuristic.
    double[] priorities = new double[graph.length];
    Arrays.fill(priorities, Double.MAX_VALUE);

    // start node has a priority equal to straight line distance to goal
    priorities[start] = heuristic(poseIndexes[start], poseIndexes[goal]);

    //track which nodes are visited
    boolean[] visited = new boolean[graph.length];

    // While there are nodes left
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

      // no paths available: return null
      if (lowestPriorityIndex == -1) {
        return null;
      } 

      // at goal node: gen path and ret
      else if (lowestPriorityIndex == goal) {
        ArrayList<Integer> ret = new ArrayList<Integer>();
        int cur = goal;
        while (cur!=start){
          ret.add(cur);
          cur=prev[cur];
        }
        ret.add(cur);
        return ret;
      }

      // for all unvisited neighboring nodes
      for (int i = 0; i < graph[lowestPriorityIndex].length; i++) {
        if (graph[lowestPriorityIndex][i] != 0 && !visited[i]) {
          // if path over this edge is shorter
          if (distances[lowestPriorityIndex] + graph[lowestPriorityIndex][i] < distances[i]) {
            // save this path as new shortest path
            distances[i] = distances[lowestPriorityIndex] + graph[lowestPriorityIndex][i];
            prev[i] = lowestPriorityIndex;

            // set the priority for the node
            priorities[i] = distances[i] + heuristic(poseIndexes[i], poseIndexes[goal]);
          }
        }
      }

      // mark as visited
      visited[lowestPriorityIndex] = true;
    }
  }

  private static ArrayList<Integer> getPathTo(int node){
    
  }

  /**
   * An estimation of distance from node x to y that is guaranteed to be lower than the actual distance
   * Currently it is simply straight-line distance
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
