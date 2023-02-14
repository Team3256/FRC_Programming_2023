// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.FieldConstants.Community.*;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import java.util.ArrayList;

public final class DynamicPathConstants {
  public static final boolean kDynamicPathGenerationDebug = true;
  public static final double kCollisionBuffer = 0.1;
  public static final double kRobotRadius = 0.455 * Math.sqrt(2);
  public static final double kHitBoxResolution = 100;
  public static final double kControlPointScalar = 0.2;

  public static final Obstacle lowerWall =
      new Obstacle(new Translation2d(kFieldLength, 0.1), kFieldLength, -0.1);
  public static final Obstacle chargingStation =
      new Obstacle(kChargingStationTopLeftCorner, kChargingStationWidth, kChargingStationHeight);
  public static final Obstacle[] obstacles = {lowerWall, chargingStation};

  public static final double kGridXLowerBound = 0;
  public static final double kGridXUpperBound = kFieldLength;

  public static final double kGridYLowerBound = 0;
  public static final double kGridYUpperBound = kFieldWidth;

  public static final double kGridXResolution = 0.70;
  public static final double kGridYResolution = 0.30;

  public static final ArrayList<Translation2d> dynamicPathAllowedPositions = new ArrayList<>();

  static {
    for (double x = kGridXLowerBound; x < kGridXUpperBound; x += kGridXResolution) {
      for (double y = kGridYLowerBound; y < kGridYUpperBound; y += kGridYResolution) {
        boolean invalidPoint = false;
        for (int i = 0; i < kHitBoxResolution; i++) {
          double angle = i * 2 * Math.PI / kHitBoxResolution;
          double padding = kRobotRadius + kCollisionBuffer;
          for (Obstacle obstacle : obstacles) {
            if (obstacle.containsPoint(
                new Translation2d(x + padding * Math.cos(angle), y + padding * Math.sin(angle)))) {
              invalidPoint = true;
              break;
            }
          }
        }
        if (!invalidPoint) dynamicPathAllowedPositions.add(new Translation2d(x, y));
      }
    }
  }

  public static final PathConstraints dynamicPathConstraints = new PathConstraints(5, 7.5);
  public static final double INF_TIME = Double.MAX_VALUE / 10;
  public static final double ILLEGAL_TIME = Double.MAX_VALUE / 20;
}
