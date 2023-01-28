// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import static frc.robot.Constants.DynamicPathGenerationConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.helpers.AStar;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.swerve.SwerveDrive;

public class DynamicPathFollowing extends CommandBase {
  private SwerveDrive swerveSubsystem;
  private Pose2d finalPose;
  private AutoCommandRunner commandRunner;
  private double[][] pathAdjacencyGraph = adjacencyGraph;
  private PathPlannerTrajectory pathToScoringLocation;

  public DynamicPathFollowing(SwerveDrive swerveSubsystem, Pose2d finalPose) {
    this.swerveSubsystem = swerveSubsystem;
    this.finalPose = finalPose;

    addRequirements(swerveSubsystem);
  }

  public DynamicPathFollowing(
      SwerveDrive swerveSubsystem, Pose2d finalPose, AutoCommandRunner commandRunner) {
    this(swerveSubsystem, finalPose);

    // TODO Implement command runner
    this.commandRunner = commandRunner;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveSubsystem.getPose();
    PathPoint currentPoint = new PathPoint(currentPose.getTranslation(), new Rotation2d(), currentPose.getRotation());

    double[][] updatedGraph = addStartAndEndNodes(pathAdjacencyGraph, currentPose.getTranslation(),
        finalPose.getTranslation());

    // The last two nodes are the start and goal nodes in the updatedGraph
    AStar.findPath(updatedGraph, heuristic, updatedGraph.length - 2, updatedGraph.length - 1);
    // pathToScoringLocation = PathPlanner.generatePath(dynamicPathConstraints,
    // currentPoint, finalPose);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    // TODO FIXME
    return false;
  }

  private double[][] addStartAndEndNodes(double[][] graph, Translation2d start, Translation2d goal) {
    int length = graph[0].length;
    // Second last index
    int startNodeIndex = length - 2 - 1;
    // Last index
    int goalNodeIndex = length - 1 - 1;

    double startNodeRow[] = new double[length - 2];
    double goalNodeRow[] = new double[length - 2];

    for (int i = 0; i < poseIndexes.length; ++i) {
      double startNodeDistance = start.getDistance(poseIndexes[i]);
      double goalNodeDistance = goal.getDistance(poseIndexes[i]);

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
