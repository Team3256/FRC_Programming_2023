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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import java.util.ArrayList;

public final class DynamicPathConstants {
  public static final boolean kDynamicPathGenerationDebug = true;
  public static final double kCollisionBuffer = 0.1;
  public static final double kRobotRadius = 0.455 * Math.sqrt(2);
  public static final double kHitBoxResolution = 100;
  public static final double kRegularControlPointScalar = 0.5087764111;
  public static final double kTightControlPointScalar = 0.1007753469;
  public static final double kStraightControlPointScalar = 0.9999813901;

  public static final Obstacle kBarrierAboveGrid =
      new Obstacle(new Translation2d(0, 5.48), 3.27, 0.02);
  public static final Obstacle kLowerWall = new Obstacle(new Translation2d(0, 0), kFieldLength, 0);

  public static final Obstacle kChargingStation =
      new Obstacle(
          kBlueChargingStationTopLeftCorner, kChargingStationWidth, kChargingStationHeight);

  public static final Obstacle[] obstacles = {
    kBarrierAboveGrid,
    kLowerWall,
    kChargingStation,
    kChargingStation.getRedVersion(),
    kLowerWall.getRedVersion(),
    kBarrierAboveGrid.getRedVersion()
  };

  public static final double kGridXLowerBound = 1.45 + kRobotRadius;
  public static final double kGridXUpperBound = kFieldLength;

  public static final double kGridYLowerBound = 0;
  public static final double kGridYUpperBound = kFieldWidth;

  public static final double kGridXResolution = 0.7; // original res 0.7
  public static final double kGridYResolution = 0.2; // original res 0.2

  public static final ArrayList<PathNode> dynamicPathWayNodes = new ArrayList<>();

  public static final boolean blue = false;

  static {
    CreateDynamicPathWayNodes.init();
  }

  public static final PathConstraints dynamicPathConstraints = new PathConstraints(5, 7.5);
  public static final double INF_TIME = Double.MAX_VALUE / 10;
  public static final double ILLEGAL_TIME = Double.MAX_VALUE / 20;

  public static final Pose2d[] kBlueImportantLocations =
      new Pose2d[] {
        // scoring locations (bottom to top)
        new Pose2d(new Translation2d(1.94, 0.61), Rotation2d.fromDegrees(-174.75)),
        new Pose2d(new Translation2d(1.98, 1.04), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.98, 1.61), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.98, 2.12), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.98, 2.71), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.98, 3.24), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.98, 3.82), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.98, 4.35), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(2.05, 4.81), Rotation2d.fromDegrees(174)),
        // loading locations (dummy)
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(21)),
        // charging station locations (dummy)
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(21)),
      };
}
