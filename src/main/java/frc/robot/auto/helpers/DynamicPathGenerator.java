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
import java.util.List;

public class DynamicPathGenerator {
  private final Pose2d startPose;
  private final Pose2d goalPose;
  private AutoCommandRunner commandRunner;
  private double[][] pathAdjacencyGraph = adjacencyGraph;
  private PathPlannerTrajectory pathToScoringLocation;

  public DynamicPathGenerator(Pose2d startPose, Pose2d goalPose) {
    this.startPose = startPose;
    this.goalPose = goalPose;
  }

  public List<Pose2d> computePath() {
    double[][] updatedGraph =
        addStartAndEndNodes(
            pathAdjacencyGraph, startPose.getTranslation(), goalPose.getTranslation());

    ArrayList<Pose2d> poses = new ArrayList<>();
    for (Pose2d pose : poseIndexes) poses.add(pose);
    poses.add(startPose);
    poses.add(goalPose);
    // The last two nodes are the start and goal nodes in the updatedGraph
    DynamicPathFinder pathFinder =
        new DynamicPathFinder(
            updatedGraph, updatedGraph.length - 2, updatedGraph.length - 1, poses);

    // Path List
    List<Pose2d> ret = new ArrayList<>();
    ArrayList<Integer> pathIndexes = pathFinder.findPath();
    for (int index : pathIndexes) {
      ret.add(poseIndexes[index]);
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

  private double[][] addStartAndEndNodes(
      double[][] graph, Translation2d start, Translation2d goal) {
    int length = graph[0].length;
    // Second last index
    int startNodeIndex = length - 2 - 1;
    // Last index
    int goalNodeIndex = length - 1 - 1;

    double startNodeRow[] = new double[length - 2];
    double goalNodeRow[] = new double[length - 2];

    for (int i = 0; i < poseIndexes.length; ++i) {
      double startNodeDistance = start.getDistance(poseIndexes[i].getTranslation());
      double goalNodeDistance = goal.getDistance(poseIndexes[i].getTranslation());

      startNodeRow[i] = startNodeDistance;
      graph[i][startNodeIndex] = startNodeDistance;

      goalNodeRow[i] = goalNodeDistance;
      graph[i][goalNodeIndex] = goalNodeDistance;
    }

    graph[startNodeIndex] = startNodeRow;
    graph[goalNodeIndex] = goalNodeRow;

    return graph;
  }
}
