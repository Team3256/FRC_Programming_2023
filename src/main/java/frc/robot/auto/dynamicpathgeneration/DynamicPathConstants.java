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
  public static final double kControlPointScalar = 0.15;

  public static final Obstacle kBarrierAboveGrid =
      new Obstacle(new Translation2d(0, 5.35), 3.27, 0.41);
  public static final Obstacle kLowerWall =
      new Obstacle(new Translation2d(0, -0.1), kFieldLength, 0.1);

  public static final Obstacle kChargingStation =
      new Obstacle(kChargingStationTopLeftCorner, kChargingStationWidth, kChargingStationHeight);
  public static final Obstacle[] obstacles = {kBarrierAboveGrid, kLowerWall, kChargingStation};

  public static final double kGridXLowerBound = 1.45 + kRobotRadius;
  public static final double kGridXUpperBound = kFieldLength;

  public static final double kGridYLowerBound = 0;
  public static final double kGridYUpperBound = kFieldWidth;

  public static final double kGridXResolution = 0.7; // original res 0.7
  public static final double kGridYResolution = 0.2; // original res 0.2

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
