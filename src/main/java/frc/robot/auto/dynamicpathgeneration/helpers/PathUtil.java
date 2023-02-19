// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathFinder.doesPathSegmentHitObstacles;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerve.SwerveConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.PriorityQueue;

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
}
