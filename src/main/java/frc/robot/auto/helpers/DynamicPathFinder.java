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
import frc.robot.swerve.SwerveConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * CREDIT FOR CODE: <a href="https://www.algorithms-and-technologies.com/a_star/java/">...</a>
 * Adapted for Team 3256's needs Used to perform the A-Star (A*) Algorithm to find the shortest path
 * from a start to a target node.
 */
public class DynamicPathFinder {
  double[][] graph;
  int src;
  int sink;

  double[] dist;
  int[] pre;
  double[] priority;

  static double INF = Double.MAX_VALUE / 2;

  /**
   * Finds the shortest distance between two nodes using Warrior-star algorithm
   *
   * @param graph an adj matrix: graph[i][j] is cost from i->j, if graph[i][j] is 0 then no edge
   * @param src start node
   * @param sink end node
   */
  public DynamicPathFinder(double[][] graph, int src, int sink) {
    this.src = src;
    this.sink = sink;
    this.graph = graph;
  }

  public ArrayList<Integer> findPath() {
    // This contains the time to travel from src to to all other nodes
    dist = new double[graph.length];
    Arrays.fill(dist, Double.MAX_VALUE);
    dist[src] = 0;

    // Previous node in maximal math to node
    pre = new int[graph.length];

    // priorities with which to visit the nodes
    priority = new double[graph.length];
    Arrays.fill(priority, Double.MAX_VALUE);
    priority[src] = heuristic(poseIndexes[src], poseIndexes[sink]);

    // which nodes are visited
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

      // at goal node: return the path stored to it
      else if (cur == sink) {
        return getStoredPathTo(sink);
      }

      // update all unvisited neighboring nodes
      for (int node = 0; node < graph[cur].length; node++) {
        ArrayList<Integer> path = getStoredPathTo(cur);
        if (graph[cur][node] != 0 && !vis[node]) {
          path.add(node);
          // if path over this edge is shorter
          if (getPathTime(path) < dist[node]) {
            // save path as new shortest path
            dist[node] = dist[cur] + graph[cur][node];
            pre[node] = cur;

            // update priority for the node
            priority[node] = dist[node] + heuristic(poseIndexes[node], poseIndexes[sink]);
          }
          path.remove(path.size() - 1);
        }
      }

      // mark as visited
      vis[cur] = true;
    }
  }

  private double getPathTime(ArrayList<Integer> path) {
    List<PathPoint> waypoints = new ArrayList<>();
    for (int node : path) waypoints.add(new PathPoint(poseIndexes[node].getTranslation(), poseIndexes[node].getRotation()));
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(dynamicPathConstraints, waypoints);
    return trajectory.getTotalTimeSeconds();
  }

  private ArrayList<Integer> getStoredPathTo(int node) {
    ArrayList<Integer> ret = new ArrayList<>();
    int cur = node;
    while (cur != src) {
      ret.add(cur);
      cur = pre[cur];
    }
    ret.add(cur);
    Collections.reverse(ret);
    return ret;
  }

  /**
   * estimate cost that is guaranteed to be lower than the actual distance Currently it is time to
   * travel euclidean distance, without allowing illegal moves
   */
  public static double heuristic(Pose2d pose1, Pose2d pose2) {
    if (isPathConnectionValid(pose1, pose2))
      return pose1.getTranslation().getDistance(pose2.getTranslation()) / SwerveConstants.kMaxSpeed;
    else return INF;
  }

  public static boolean isPathConnectionValid(Pose2d pose1, Pose2d pose2) {
    Translation2d translation1 = pose1.getTranslation();
    Translation2d translation2 = pose2.getTranslation();

    for (Translation2d[] chargingStationCorner :
        FieldConstants.Community.kChargingStationSegments) {
      if (lineSegmentsIntersecting(
          chargingStationCorner[0], chargingStationCorner[1], translation1, translation2)) {
        return false;
      }
    }
    return true;
  }

  public static boolean lineSegmentsIntersecting(
      Translation2d start1, Translation2d end1, Translation2d start2, Translation2d end2) {
    // TODO: add buffer
    if (start1.getX() > end1.getX()) {
      if ((start1.getX() > start2.getX() && start2.getX() > end1.getX())
          || (start1.getX() > end2.getX() && end2.getX() > end1.getX())) return true;
    } else {
      if ((end1.getX() > start2.getX() && start2.getX() > start1.getX())
          || (end1.getX() > end2.getX() && end2.getX() > start1.getX())) return true;
    }

    if (start2.getX() > end2.getX()) {
      if ((start2.getX() > start1.getX() && start1.getX() > end2.getX())
          || (start2.getX() > end1.getX() && end1.getX() > end2.getX())) return true;
    } else {
      if ((end2.getX() > start1.getX() && start1.getX() > start2.getX())
          || (end2.getX() > end1.getX() && end1.getX() > start2.getX())) return true;
    }

    return false;
  }
}
