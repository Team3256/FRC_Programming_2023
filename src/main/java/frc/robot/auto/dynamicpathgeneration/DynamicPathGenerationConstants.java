// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.FieldConstants.Community.*;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DynamicPathGenerationConstants {
  public static final boolean kDynamicPathGenerationDebug = true;
  public static final double kCollisionBuffer = Units.inchesToMeters(10);
  public static final double kControlPointScalar = 0.2;
  public static final Translation2d[][] kChargingStationSegments =
      new Translation2d[][] {
        {kChargingStationCorners[0], kChargingStationCorners[1]},
        {kChargingStationCorners[0], kChargingStationCorners[2]},
        {kChargingStationCorners[1], kChargingStationCorners[3]},
        {kChargingStationCorners[2], kChargingStationCorners[3]},
      };

  // Graph represnted below in (x, y)
  // <-> Represent edges
  // Graph is circular (first on top and bottom line connect, last on each line
  // connect)
  // 5.9, 4.3 <-> 4.5, 4.65 <-> 3.3, 4.65 <-> 2.2, 4.4 <-> 2.2, 2.7
  // |
  // 5.9, 1.3 <-> 4.5, 0.70 <-> 3.3, 0.70 <-> 2.2, 1.0 <-> 2.2, 2.7
  public static final Pose2d poseIndexes[] =
      new Pose2d[] {
        new Pose2d(new Translation2d(5.9, 4.3), Rotation2d.fromDegrees(-35)),
        new Pose2d(new Translation2d(4.5, 4.65), Rotation2d.fromDegrees(0)),
        new Pose2d(new Translation2d(3.3, 4.65), Rotation2d.fromDegrees(-170)),
        new Pose2d(new Translation2d(2.2, 4.4), Rotation2d.fromDegrees(-125)),
        new Pose2d(new Translation2d(2.2, 2.7), Rotation2d.fromDegrees(90)),
        new Pose2d(new Translation2d(5.9, 1.3), Rotation2d.fromDegrees(40)),
        new Pose2d(new Translation2d(4.5, 0.7), Rotation2d.fromDegrees(0)),
        new Pose2d(new Translation2d(3.3, 0.7), Rotation2d.fromDegrees(-10)),
        new Pose2d(new Translation2d(2.2, 1.0), Rotation2d.fromDegrees(-80)),
      };

  // Will be filled by start and end pose
  private static final double[] emptyRow = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  public static final double[][] origGraph =
      new double[][] {
        {0.0000, 1.4431, 0.0000, 0.0000, 0.0000, 3.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000},
        {1.4431, 0.0000, 1.2000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000},
        {0.0000, 1.2000, 0.0000, 1.1281, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000},
        {0.0000, 0.0000, 1.1281, 0.0000, 2.7000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000},
        {0.0000, 0.0000, 0.0000, 2.7000, 0.0000, 0.0000, 0.0000, 0.0000, 1.7000, 0.0000, 0.0000},
        {3.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.5232, 0.0000, 0.0000, 0.0000, 0.0000},
        {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.5232, 0.0000, 1.2000, 0.0000, 0.0000, 0.0000},
        {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.2000, 0.0000, 1.1402, 0.0000, 0.0000},
        {0.0000, 0.0000, 0.0000, 0.0000, 1.7000, 0.0000, 0.0000, 1.1402, 0.0000, 0.0000, 0.0000},
        emptyRow.clone(),
        emptyRow.clone()
      };

  public static final PathConstraints dynamicPathConstraints = new PathConstraints(5, 5);
}
