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
  ArrayList<Pose2d> poses;

  double[] dist;
  int[] pre;
  double[] priority;

  static double INF = Double.MAX_VALUE / 10;
  boolean debug = true;

  /**
   * Finds the shortest distance between two nodes using Warrior-star algorithm
   *
   * @param graph an adj matrix: graph[i][j] is cost from i->j, if graph[i][j] is 0 then no edge
   * @param src start node
   * @param sink end node
   */
  public DynamicPathFinder(double[][] graph, int src, int sink, ArrayList<Pose2d> poses) {
    this.src = src;
    this.sink = sink;
    this.graph = graph;
    this.poses = poses;
  }

  public ArrayList<Integer> findPath() {
    // Time to travel from src to all other nodes
    dist = new double[graph.length];
    Arrays.fill(dist, INF);
    dist[src] = 0;

    // Previous node in current optimal path to node
    pre = new int[graph.length];

    // Priorities with which to visit the nodes
    priority = new double[graph.length];
    Arrays.fill(priority, INF);
    priority[src] = heuristic(poses.get(src), poses.get(sink));

    // Visited nodes
    boolean[] vis = new boolean[graph.length];

    while (true) {
      // Find unvisited lowest priority node
      double curPriority = INF;
      int cur = -1;
      for (int node = 0; node < priority.length; node++) {
        if (priority[node] < curPriority && !vis[node]) {
          curPriority = priority[node];
          cur = node;
        }
      }
      if (debug) {
        System.out.println("cur:" + cur);
      }
      // No paths available
      if (cur == -1) {
        ArrayList<Integer> ret = new ArrayList<>();
        ret.add(graph.length - 2);
        ret.add(graph.length - 1);
        return ret;
      }

      // Found shortest path to sink
      else if (cur == sink) {
        if (debug) System.out.println("Done!");
        return getStoredPathTo(sink);
      }

      // Update all unvisited neighboring nodes
      for (int node = 0; node < graph[cur].length; node++) {
        ArrayList<Integer> path = getStoredPathTo(cur);
        if (poses.get(node).getX() < poses.get(cur).getX() && !vis[node]) {
          path.add(node);
          double pathTime = getPathTime(path);
          // If path over this edge is better
          if (debug) {
            System.out.println("try:" + node);
            System.out.println("path:" + path);
            System.out.println("pathTime:" + pathTime);
          }
          if (pathTime < dist[node]) {
            // Save path as new current shortest path
            dist[node] = pathTime;
            pre[node] = cur;

            // Update node priority
            priority[node] = dist[node] + heuristic(poses.get(node), poses.get(sink));

            if (debug) {
              System.out.println("next:" + node);
              System.out.println("dist:" + dist[node]);
              System.out.println("heuristic:" + heuristic(poses.get(node), poses.get(sink)));
              System.out.println("priority:" + priority[node]);
            }
          }
          path.remove(path.size() - 1);
        }
      }

      // mark as visited
      vis[cur] = true;
    }
  }

  private double getPathTime(ArrayList<Integer> path) {
    for (int i = 0; i < path.size() - 1; i++) {
      if (!isPathConnectionValid(poses.get(path.get(i)), poses.get(path.get(i + 1)))) return INF;
    }
    List<PathPoint> waypoints = new ArrayList<>();
    for (int node : path)
      waypoints.add(new PathPoint(poses.get(node).getTranslation(), poses.get(node).getRotation()));
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
    // if (isPathConnectionValid(pose1, pose2))
    return pose1.getTranslation().getDistance(pose2.getTranslation()) / SwerveConstants.kMaxSpeed;
    // else return INF;
  }

  public static boolean isPathConnectionValid(Pose2d pose1, Pose2d pose2) {
    Translation2d translation1 = pose1.getTranslation();
    Translation2d translation2 = pose2.getTranslation();

    for (Translation2d[] chargingStationCorner :
        FieldConstants.Community.kChargingStationSegments) {
      if (doIntersect(
          chargingStationCorner[0], chargingStationCorner[1], translation1, translation2)) {
        System.out.println("Pose1:" + pose1 + ", Pose2:" + pose2 + " FAIL");
        return false;
      }
    }
    return true;
  }

  static int orientation(Translation2d p, Translation2d q, Translation2d r) {
    double val =
        (q.getY() - p.getY()) * (r.getX() - q.getX())
            - (q.getX() - p.getX()) * (r.getY() - q.getY());

    return (val > 0) ? 1 : 2; // CW/CCW
  }

  public static boolean doIntersect(
      Translation2d start1, Translation2d end1, Translation2d start2, Translation2d end2) {
    int o1 = orientation(start1, end1, start2);
    int o2 = orientation(start1, end1, end2);
    int o3 = orientation(start2, end2, start1);
    int o4 = orientation(start2, end2, end1);

    return (o1 != o2 && o3 != o4);
  }
}
