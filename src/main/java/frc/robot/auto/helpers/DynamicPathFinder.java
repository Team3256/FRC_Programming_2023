// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.Constants.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerve.SwerveConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * CREDIT FOR CODE: https://www.algorithms-and-technologies.com/a_star/java/ Adapted for Team 3256's
 * needs Used to perform the A-Star (A*) Algorithm to find the shortest path from a start to a
 * target node.
 */
public class DynamicPathFinder {
  double[][] graph;
  int src;
  int sink;

  double[] dist;
  int[] pre;
  double[] priority;
  /**
   * Finds the shortest distance between two nodes using Warrior-star algorithm
   * @param graph an adj matrix: graph[i][j] is cost from i->j, if graph[i][j] is 0 then no edge
   * @param src start node
   * @param sink end node
   */
  public DynamicPathFinder (double[][] graph, int src, int sink){
    this.src=src;
    this.sink=sink;
    this.graph=graph;
  }
  public ArrayList<Integer> findPath() {
    // This contains the time to travel from src to to all other nodes
    dist= new double[graph.length];
    pre = new int[graph.length];

    // Initializing with a distance of "Infinity"
    Arrays.fill(dist, Double.MAX_VALUE);
    // The distance from the start node to itself is of course 0
    dist[src] = 0;

    // This contains the priorities with which to visit the nodes, calculated using the heuristic.
    priority = new double[graph.length];
    Arrays.fill(priority, Double.MAX_VALUE);

    // start node has a priority equal to straight line distance to goal
    priority[src] = heuristic(poseIndexes[src], poseIndexes[sink]);

    //track which nodes are visited
    boolean[] vis = new boolean[graph.length];

    // run until reached termination state
    while (true) {
      // find unvisited lowest priority node
      double curPriority = Integer.MAX_VALUE;
      int cur = -1;
      for (int node = 0; node < priority.length; node++) {
        if (priority[node] < curPriority && !vis[node]) {
          curPriority = priority[node];
          cur = node;
        }
      }

      // no paths available: return null
      if (cur == -1) {
        return null;
      } 

      // at goal node: generate path and return it
      else if (cur == sink) {
        return getBestPathTo(sink);
      }

      // update all unvisited neighboring nodes
      for (int node = 0; node < graph[cur].length; node++) {
        if (graph[cur][node] != 0 && !vis[node]) {
          // if path over this edge is shorter
          if (dist[cur] + graph[cur][node] < dist[node]) {
            // save this path as new shortest path
            dist[node] = dist[cur] + graph[cur][node];
            pre[node] = cur;

            // set the priority for the node
            priority[node] = dist[node] + heuristic(poseIndexes[node], poseIndexes[sink]);
          }
        }
      }

      // mark as visited
      vis[cur] = true;
    }
  }

  private double getPathTime(ArrayList<Integer> path){
    List<PathPoint> waypoints = new ArrayList<>();
    for (int node : path) waypoints.add(new PathPoint(poseIndexes[node].getTranslation(), poseIndexes[node].getRotation()));
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(dynamicPathConstraints, waypoints);
    return trajectory.getTotalTimeSeconds();
  }
  
  private ArrayList<Integer> getBestPathTo(int node){
    ArrayList<Integer> ret = new ArrayList<>();
    int cur = node;
    while (cur!=src){
      ret.add(cur);
      cur=pre[cur];
    }
    ret.add(cur);
    return ret;
  }

  /**
   * estimate time p1->p2, lower than real
   * currently simply time to travel euclidean dist
   *  */  
  private static double heuristic(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation())/SwerveConstants.kMaxSpeed;
  }

  //TODO: Implement method to detect if there is an obstacle in between 2 poses
  private static boolean isPathConnectionValid(Pose2d pose1, Pose2d pose2) {
    Translation2d translation1 = pose1.getTranslation();
    Translation2d translation2 = pose2.getTranslation();
    return true;
  }
}
