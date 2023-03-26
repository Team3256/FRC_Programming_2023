// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.swerve.SwerveConstants;
import java.util.ArrayList;

public class PathUtil {
  public static double straightTravelTimeWithoutObstacles(
      Translation2d position1, Translation2d position2) {
    return position1.getDistance(position2) / (SwerveConstants.kMaxSpeed);
  }

  public static double straightTravelTimeWithObstacles(
      Translation2d position1, Translation2d position2) {
    double estimatedTime = straightTravelTimeWithoutObstacles(position1, position2);
    if (doesPathSegmentHitObstacles(position1, position2)) estimatedTime += ILLEGAL_TIME;
    return estimatedTime;
  }

  public static boolean doesLineHitObstacles(Translation2d position1, Translation2d position2) {
    for (Obstacle obstacle : obstacles) {
      if (obstacle.intersectsLineSegment(position1, position2)) return true;
    }
    return false;
  }

  public static boolean doesPathSegmentHitObstacles(
      Translation2d position1, Translation2d position2) {
    if (doesLineHitObstacles(position1, position2)) return true;

    Rotation2d normalAngle = position2.minus(position1).getAngle().plus(Rotation2d.fromDegrees(90));
    Translation2d normalVector = new Translation2d(1, 0).rotateBy(normalAngle).times(kRobotRadius);

    if (doesLineHitObstacles(position1.minus(normalVector), position2.minus(normalVector)))
      return true;
    if (doesLineHitObstacles(position1.plus(normalVector), position2.plus(normalVector)))
      return true;

    return false;
  }

  public static Pose2d flip(Pose2d original) {
    return new Pose2d(
        Constants.FieldConstants.kFieldLength - original.getX(),
        original.getY(),
        original.getRotation().unaryMinus().plus(Rotation2d.fromDegrees(180)));
  }

  public static Translation2d flip(Translation2d orig) {
    return new Translation2d(Constants.FieldConstants.kFieldLength - orig.getX(), orig.getY());
  }

  // helper methods to connect/disconnect edges
  public static void fullyConnect(ArrayList<PathNode> pathNodes) {
    for (PathNode u : pathNodes) {
      for (PathNode v : pathNodes) {
        if (u != v) fullyConnect(u, v);
      }
    }
  }

  public static void fullyConnect(ArrayList<PathNode> pathNodes1, ArrayList<PathNode> pathNodes2) {
    for (PathNode u : pathNodes1) {
      for (PathNode v : pathNodes2) {
        fullyConnect(u, v);
      }
    }
  }

  public static void fullyConnect(PathNode u, ArrayList<PathNode> pathNodes) {
    for (PathNode v : pathNodes) {
      fullyConnect(u, v);
    }
  }

  public static void fullyConnect(PathNode u, PathNode v) {
    u.addEdge(v.getIndex());
    v.addEdge(u.getIndex());
  }

  public static void fullyDisconnect(PathNode u, PathNode v) {
    u.remEdge(v.getIndex());
    v.remEdge(u.getIndex());
  }

  public static void fullyDisconnect(PathNode u, ArrayList<PathNode> pathNodes) {
    for (PathNode v : pathNodes) {
      fullyDisconnect(u, v);
    }
  }
}
