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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import java.util.ArrayList;

public final class DynamicPathConstants {
  // Flags
  public static final boolean kDynamicPathGenerationDebug = true && Constants.kDebugEnabled;

  // Bezier
  public static final double kRegularControlPointScalar = 0.5;
  public static final double kBetweenPreSinkPointScalar = 0.1;
  public static final double kBetweenPassageControlPointScalar = 0.90;

  // Obstacles
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

  // Universal path nodes
  public static final ArrayList<PathNode> blueDynamicPathWayNodes = new ArrayList<>();
  public static final ArrayList<PathNode> redDynamicPathWayNodes = new ArrayList<>();
  public static final double preSinkX = 2.10;
  public static final double preSinkEndpointsOffset = 0.3;
  public static final double passagePoints = 8;

  // TODO: Find a way for this to not be called during first command press (takes
  // 45 ms gen)
  static {
    CreateDynamicPathWayNodes.init();
  }

  // Trajectory constraints
  public static final PathConstraints kDynamicPathConstraints = new PathConstraints(1, 1);
  public static final PathConstraints kPathToScoreConstraints = new PathConstraints(1, 1);
  public static final double kGridBuffer = Units.inchesToMeters(4);
  public static final double kBlueGridTapeX = 1.38 + kGridBuffer;
  public static final double kBlueLowTapeOffset = Units.inchesToMeters(22);
  public static final double kBlueMidTapeOffset = Units.inchesToMeters(12);
  public static final double kBlueHighTapeOffset = Units.inchesToMeters(0);
  public static final double kOuterNodeRotationBuffer = Units.inchesToMeters(3);

  // (lowest y location to highest y location)
  // TODO: LOW PRIORITY - Use robust center pivot radius vector method instead of
  // mass setting (only
  // set angle for each and not this monstrous equation)
  public static final Transform2d kSubstationPreSink =
      new Transform2d(new Translation2d(-1.22, 0), new Rotation2d());
  public static final Pose2d kBlueTopDoubleSubstationPose =
      new Pose2d(15.40, 7.35, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueBottomDoubleSubstationPose =
      new Pose2d(15.40, 6.20, Rotation2d.fromDegrees(0));
  public static final Pose2d[] kBottomBlueScoringPoses =
      new Pose2d[] {
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX
                    + kBlueLowTapeOffset
                    + kOuterNodeRotationBuffer
                    + Constants.kRobotLength / 2,
                0.69),
            Rotation2d.fromDegrees(-175)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 1.04),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 1.61),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 2.12),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 2.71),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 3.24),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 3.82),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueLowTapeOffset + Constants.kRobotLength / 2, 4.35),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX
                    + kBlueLowTapeOffset
                    + kOuterNodeRotationBuffer
                    + Constants.kRobotLength / 2,
                4.72),
            Rotation2d.fromDegrees(173))
      };

  public static final Pose2d[] kMidBlueScoringPoses =
      new Pose2d[] {
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX
                    + kBlueMidTapeOffset
                    + kOuterNodeRotationBuffer
                    + Constants.kRobotLength / 2,
                0.67),
            Rotation2d.fromDegrees(-175)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 1.04),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 1.61),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 2.12),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 2.71),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 3.24),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 3.82),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueMidTapeOffset + Constants.kRobotLength / 2, 4.35),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX
                    + kBlueMidTapeOffset
                    + kOuterNodeRotationBuffer
                    + Constants.kRobotLength / 2,
                4.75),
            Rotation2d.fromDegrees(173))
      };

  public static final Pose2d[] kHighBlueScoringPoses =
      new Pose2d[] {
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX
                    + kBlueHighTapeOffset
                    + kOuterNodeRotationBuffer
                    + Constants.kRobotLength / 2,
                0.64),
            Rotation2d.fromDegrees(-175)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 1.04),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 1.61),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 2.12),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 2.71),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 3.24),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 3.82),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX + kBlueHighTapeOffset + Constants.kRobotLength / 2, 4.35),
            Rotation2d.fromDegrees(180)),
        new Pose2d(
            new Translation2d(
                kBlueGridTapeX
                    + kBlueHighTapeOffset
                    + kOuterNodeRotationBuffer
                    + Constants.kRobotLength / 2,
                4.79),
            Rotation2d.fromDegrees(173))
      };

  // Path finding constraints
  public static final double INF_TIME = Double.MAX_VALUE / 10;
  public static final double ILLEGAL_TIME = Double.MAX_VALUE / 20;
  public static final double kRobotRadius = 0.47 * Math.sqrt(2);
}
