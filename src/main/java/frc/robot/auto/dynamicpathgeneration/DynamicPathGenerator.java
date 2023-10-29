// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.trajectoryViewer;
import static frc.robot.Constants.waypointViewer;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.dynamicpathgeneration.helpers.*;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;
import java.util.*;

public class DynamicPathGenerator {
  private final Pose2d srcPose;
  private final PathNode srcNode;
  private final Pose2d sinkPose;
  private final PathNode sinkNode;
  private final ArrayList<PathNode> dynamicPathNodes;
  private final List<Integer> pathIndexes;

  /**
   * Set up visibility graph for DPG and store the best path for the robot to take
   *
   * @param srcPose the initial pose of the robot in the path
   * @param sinkPose the final pose of the robot in the path
   */
  public DynamicPathGenerator(Pose2d srcPose, Pose2d sinkPose) {
    this.srcPose = srcPose;
    this.sinkPose = sinkPose;
    System.out.println("Initializing DPG");
    // update visibility graph, taking into account the srcPose, sinkPose, and alliance color
    dynamicPathNodes = new ArrayList<>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      dynamicPathNodes.addAll(blueDynamicPathWayNodes);
    } else {
      dynamicPathNodes.addAll(redDynamicPathWayNodes);
    }
    srcNode = new PathNode(srcPose.getX(), srcPose.getY(), PathNode.NodeType.SRC);
    sinkNode = new PathNode(sinkPose.getX(), sinkPose.getY(), PathNode.NodeType.SINK);
    // note: start node MUST be added before goal node
    dynamicPathNodes.add(srcNode);
    dynamicPathNodes.add(sinkNode);
    // connect src, sink to dynamicPathNodes
    PathNode srcClosest = connectToClosest(srcNode, dynamicPathNodes);
    PathNode sinkClosest = connectToClosest(sinkNode, dynamicPathNodes);
    // use Dynamic Path Finder to find the optimal path that DPG will utilize
    DynamicPathFinder pathFinder =
        new DynamicPathFinder(srcNode.getIndex(), sinkNode.getIndex(), dynamicPathNodes);
    pathIndexes = pathFinder.findPath();
    if (kDynamicPathGenerationDebug) {
      System.out.println("DPG path indexes:");
      System.out.println(pathIndexes);
    }
    // unconnect src, sink from dynamicPathNodes
    PathUtil.fullyDisconnect(srcClosest, sinkNode);
    PathUtil.fullyDisconnect(sinkClosest, sinkNode);
  }

  /**
   * connect node to the closest node in pathNodes add node to pathNodes
   *
   * @param node node to connect to pathNodes
   * @param pathNodes list of nodes that you want to try connecting node to
   * @return
   */
  public PathNode connectToClosest(PathNode node, ArrayList<PathNode> pathNodes) {
    double closest = INF_TIME;
    PathNode ret = node;
    for (PathNode q : pathNodes) {
      if (q == node) continue;
      double dist = PathUtil.straightTravelTimeWithObstacles(node.getPoint(), q.getPoint());
      if (dist > 0.000001 && dist < closest) {
        closest = dist;
        ret = q;
      }
    }
    PathUtil.fullyConnect(ret, node);
    return ret;
  }

  public List<PathNode> getPathNodes() {
    // get list of stored pathIds from list of pathNodes
    List<PathNode> pathNodes = new ArrayList<>();
    for (int i : pathIndexes) {
      pathNodes.add(dynamicPathNodes.get(i));
    }
    return pathNodes;
  }

  public PathPlannerTrajectory getTrajectory() {
    List<PathNode> pathNodes = getPathNodes();
    // case: no path nodes were found then there should be no trajectory
    if (pathNodes.size() == 0) return null;

    // otherwise convert path nodes into path points
    Path path = new Path(getPathNodes(), srcPose.getRotation(), sinkPose.getRotation());
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }
    // convert path points into the trajectory we return
    return PathPlanner.generatePath(kWaypointPathConstraints, pathPoints);
  }

  public Command getCommand(SwerveDrive swerveDrive) {
    double ms0 = System.currentTimeMillis();
    PathPlannerTrajectory trajectory = getTrajectory();
    System.out.println("PROGRAM TOOK:" + (System.currentTimeMillis() - ms0) + " MS");
    // log the src, sink, trajectory
    if (kDynamicPathGenerationDebug) {
      if (trajectory != null)
        trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(trajectory);
      waypointViewer.getObject("Src").setPose(srcPose);
      waypointViewer.getObject("Sink").setPose(sinkPose);
      for (PathNode pathNode : dynamicPathNodes) {
        waypointViewer
            .getObject("Node " + pathNode.getIndex())
            .setPose(new Pose2d(pathNode.getX(), pathNode.getY(), new Rotation2d()));
      }
    }
    // case: no trajectory available
    if (trajectory == null) return new PrintCommand("ERROR: NO PATH AVAILABLE");
    // otherwise create command that runs the trajectory
    AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
    return autoBuilder.createPathPlannerCommand(trajectory, false, false);
  }
}
