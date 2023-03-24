// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.dynamicpathgeneration.helpers.*;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;
import static frc.robot.Constants.trajectoryViewer;
 import static frc.robot.Constants.waypointViewer;

import java.util.*;

public class DynamicPathGenerator {
  private final Pose2d srcPose;
  private final PathNode srcNode;
  private final Pose2d sinkPose;
  private final PathNode sinkNode;
  private final ArrayList<PathNode> dynamicPathNodes;

  /**
   * 
   * @param robotPose the current pose of the robot
   * @param sinkPose the wanted final pose of the robot
   */
  public DynamicPathGenerator(Pose2d robotPose, Pose2d sinkPose) {
    this.srcPose = robotPose;
    this.sinkPose = sinkPose;
    dynamicPathNodes = new ArrayList<>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      dynamicPathNodes.addAll(blueDynamicPathWayNodes);
    } else {
      dynamicPathNodes.addAll(redDynamicPathWayNodes);
    }
    srcNode = new PathNode(robotPose.getX(), robotPose.getY(), PathNode.NodeType.SRC);
    sinkNode = new PathNode(sinkPose.getX(), sinkPose.getY(), PathNode.NodeType.SINK);
    // start node must be added before goal node
    dynamicPathNodes.add(srcNode);
    dynamicPathNodes.add(sinkNode);
  }

  public PathNode connectToClosest(PathNode node, ArrayList<PathNode> nodes) {
    double closest = INF_TIME;
    PathNode ret = node;
    for (PathNode q : nodes) {
      if (q == node) continue;
      double dist = PathUtil.straightTravelTimeWithObstacles(node.getPoint(), q.getPoint());
      if (dist < closest) {
        closest = dist;
        ret = q;
      }
    }
    if (kDynamicPathGenerationDebug) {
      System.out.println("closest to " + node + " is " + ret);
    }
    PathUtil.fullyConnect(ret, node);
    return ret;
  }

  public List<Integer> getPathIds() {
    // PathNode srcClosest = connectToClosest(dynamicPathNodes.get(src),
    // dynamicPathNodes);
    PathUtil.fullyConnect(srcNode, blueDynamicPathWayNodes);
    PathNode sinkClosest = connectToClosest(sinkNode, dynamicPathNodes);
    if (kDynamicPathGenerationDebug) {
      System.out.println("src edges:" + srcNode.getEdges().size());
      System.out.println("sink edges:" + sinkNode.getEdges().size());
    }
    DynamicPathFinder pathFinder =
        new DynamicPathFinder(srcNode.getIndex(), sinkNode.getIndex(), dynamicPathNodes);
    List<Integer> pathIndexes = pathFinder.findPath();
    if (kDynamicPathGenerationDebug) {
      System.out.println("These are the path indexes:");
      System.out.println(pathIndexes);
    }
    // PathUtil.fullyDisconnect(srcClosest, dynamicPathNodes.get(src));
    PathUtil.fullyDisconnect(sinkClosest, sinkNode);
    return pathIndexes;
  }

  public List<PathNode> getPathNodes() {
    // convert pathIds into pathNodes
    List<Integer> pathIds = getPathIds();
    List<PathNode> pathNodes = new ArrayList<>();
    for (int i : pathIds) {
      pathNodes.add(dynamicPathNodes.get(i));
    }
    return pathNodes;
  }

  public PathPlannerTrajectory getTrajectory() {
    Path path = new Path(getPathNodes(), srcPose.getRotation(), sinkPose.getRotation());
    List<PathPoint> pathPoints = new ArrayList<>();
    for (Waypoint waypoint : path.getWaypoints()) {
      pathPoints.add(waypoint.waypointToPathPoint());
    }
    // if no path points were found then there should be no trajectory
    if (pathPoints.size() == 0) return null;
    // convert pathPoints into Trajectory we return
    return PathPlanner.generatePath(kDynamicPathConstraints, pathPoints);
  }

  public Command getCommand(SwerveDrive swerveDrive, PathConstraints pathConstraints) {
     PathPlannerTrajectory trajectory = getTrajectory();

     // send trajectory to networktables for logging
     if (kDynamicPathGenerationDebug) {
       trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(trajectory);
       waypointViewer.getObject("Src").setPose(trajectory.getInitialHolonomicPose());
       waypointViewer.getObject("Sink").setPose(trajectory.getEndState().poseMeters);
     }

     // create command that runs the trajectory
     AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
     Command trajCommand = autoBuilder.createPathPlannerCommand(trajectory, false, false);
     return trajCommand;
  }
}
