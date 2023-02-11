// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.auto.AutoConstants.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final Pose2d goalPose;
  private double[][] graph;
  static boolean debug = true;

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose) {
    this.startPose = startPose;
    this.goalPose = goalPose;
    graph = new double[origGraph.length][origGraph[0].length];
    for (int i = 0; i < origGraph.length; i++) graph[i] = origGraph[i].clone();
  }

  public List<Pose2d> computePath() {
    addStartAndEndNodes(graph, startPose.getTranslation(), goalPose.getTranslation());

    ArrayList<Pose2d> poses = new ArrayList<>();
    for (Pose2d pose : poseIndexes) poses.add(pose);
    poses.add(startPose);
    poses.add(goalPose);

    if (debug) {
      System.out.println("Poses:");
      System.out.println(poses);
      System.out.println("Graph:");
      for (int r = 0; r < graph.length; r++) {
        System.out.println(Arrays.toString(graph[r]));
      }
    }
    // The last two nodes are the start and goal nodes in the updatedGraph
    DynamicPathFinder pathFinder =
        new DynamicPathFinder(graph, graph.length - 2, graph.length - 1, poses);

    // Path List
    List<Pose2d> ret = new ArrayList<>();
    ArrayList<Integer> pathIndexes = pathFinder.findPath();
    for (int index : pathIndexes) {
      ret.add(poses.get(index));
    }
    if (debug) {
      System.out.println("pathIndexes:");
      System.out.println(pathIndexes);
    }
    return ret;
  }

  public PathPlannerTrajectory computeTrajectory() {
    List<Pose2d> path = computePath();
    List<PathPoint> waypoints = new ArrayList<>();
    waypoints.add(
        new PathPoint(startPose.getTranslation(), new Rotation2d(), startPose.getRotation()));
    for (Pose2d pointPose : path) {
      waypoints.add(new PathPoint(pointPose.getTranslation(), pointPose.getRotation()));
    }
    waypoints.add(
        new PathPoint(goalPose.getTranslation(), new Rotation2d(), goalPose.getRotation()));
    return PathPlanner.generatePath(dynamicPathConstraints, waypoints);
  }

  private void addStartAndEndNodes(double[][] graph, Translation2d start, Translation2d goal) {
    int length = graph[0].length;
    int startNodeIndex = length - 2;
    int goalNodeIndex = length - 1;
    for (int i = 0; i < poseIndexes.length - 2; i++) {
      double startNodeDistance = start.getDistance(poseIndexes[i].getTranslation());
      double goalNodeDistance = goal.getDistance(poseIndexes[i].getTranslation());
      graph[startNodeIndex][i] = startNodeDistance;
      graph[i][goalNodeIndex] = goalNodeDistance;
    }
  }
}
