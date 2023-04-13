// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.FieldConstants.Community.*;
import static frc.robot.Constants.FieldConstants.Grids.kBlueNodeY;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import java.util.ArrayList;

public final class DynamicPathConstants {
  // Flags
  public static final boolean kDynamicPathGenerationDebug = true && Constants.kDebugEnabled;
  public static final boolean kDynamicPathGenerationEnabled = false;

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
    if (FeatureFlags.kDynamicPathGenEnabled) CreateDynamicPathWayNodes.init();
  }

  public static final PathConstraints kWaypointPathConstraints = new PathConstraints(3, 3);
  public static final PathConstraints kPathToDestinationConstraints = new PathConstraints(2, 2);

  // x value for the blue tape in front of the blue scoring grid
  public static final double kBlueGridTapeX = 1.36;

  // x value offsets for the distance between the robots bumper and the blue tape
  // depending on the scoring height
  public static final double kBlueLowOffset = Units.inchesToMeters(22);
  public static final double kBlueMidOffset = Units.inchesToMeters(21.5);
  public static final double kBlueHighOffset = Units.inchesToMeters(0);
  public static final double kIntakeWidthOffset = Units.inchesToMeters(0); // Operator skill issue

  // Our arm is bent :)
  public static final Rotation2d kArmFckConstant = Rotation2d.fromDegrees(3);

  // x value offset for distance between the robots bumper and the blue tape
  // to determine the scoring waypoint of the robot given the scoring location
  public static final double kBlueScoringWaypointOffset = Units.inchesToMeters(14);

  // treat the edge nodes specially
  public static final double kRotationOffset = 0;
  public static final double kOuterNodeRotationBuffer = Units.inchesToMeters(0);

  // (lowest y location to highest y location)
  public static final double kSubstationWaypointOffset = Units.feetToMeters(5);
  public static final Pose2d kBlueOuterDoubleSubstationPose =
      new Pose2d(15.45, 7.35, Rotation2d.fromDegrees(0));
  public static final Pose2d kBlueInnerDoubleSubstationPose =
      new Pose2d(15.45, 6.10, Rotation2d.fromDegrees(0));
  public static final Transform2d kSubstationPreSink =
      new Transform2d(new Translation2d(-kSubstationWaypointOffset, 0), new Rotation2d());

  public static final GamePiece[] kScoringLocationPiece =
      new GamePiece[] {
        GamePiece.CONE,
        GamePiece.CUBE,
        GamePiece.CONE,
        GamePiece.CONE,
        GamePiece.CUBE,
        GamePiece.CONE,
        GamePiece.CONE,
        GamePiece.CUBE,
        GamePiece.CONE
      };
  public static final Pose2d[] kBlueScoreWaypointPoses = new Pose2d[9];
  public static final Pose2d[] kBottomBlueScoringPoses = new Pose2d[9];
  public static final Pose2d[] kMidBlueScoringPoses = new Pose2d[9];
  public static final Pose2d[] kHighBlueScoringPoses = new Pose2d[9];

  static {
    for (int i = 0; i < 9; i++) {
      kBlueScoreWaypointPoses[i] =
          new Pose2d(
              new Translation2d(
                  kBlueGridTapeX + kBlueScoringWaypointOffset + Constants.kRobotLength / 2,
                  kBlueNodeY[i] + kIntakeWidthOffset),
              Rotation2d.fromDegrees(180).plus(kArmFckConstant));
      kBottomBlueScoringPoses[i] =
          new Pose2d(
              new Translation2d(
                  kBlueGridTapeX + kBlueLowOffset + Constants.kRobotLength / 2,
                  kBlueNodeY[i] + kIntakeWidthOffset),
              Rotation2d.fromDegrees(180).plus(kArmFckConstant));
      kMidBlueScoringPoses[i] =
          new Pose2d(
              new Translation2d(
                  kBlueGridTapeX + kBlueMidOffset + Constants.kRobotLength / 2,
                  kBlueNodeY[i] + kIntakeWidthOffset),
              Rotation2d.fromDegrees(180).plus(kArmFckConstant));
      kHighBlueScoringPoses[i] =
          new Pose2d(
              new Translation2d(
                  kBlueGridTapeX + kBlueHighOffset + Constants.kRobotLength / 2,
                  kBlueNodeY[i] + kIntakeWidthOffset),
              Rotation2d.fromDegrees(180).plus(kArmFckConstant));
    }
  }

  // Path finding constraints
  public static final double INF_TIME = Double.MAX_VALUE / 10;
  public static final double ILLEGAL_TIME = Double.MAX_VALUE / 20;
  public static final double kRobotRadius = 0.47 * Math.sqrt(2);
}
