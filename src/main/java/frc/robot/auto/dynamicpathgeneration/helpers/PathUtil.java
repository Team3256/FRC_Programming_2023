// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerve.SwerveConstants;

public class PathUtil{
  public static double splineHeuristic(Translation2d position1, Translation2d position2) {
    double estimatedTime = position1.getDistance(position2) / (SwerveConstants.kMaxSpeed);
    return estimatedTime;
  }

  public static double mockSplineHeuristic(Translation2d position1, Translation2d position2) {
    double estimatedTime = splineHeuristic(position1, position2);
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
}
