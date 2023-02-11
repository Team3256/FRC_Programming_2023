// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class AutoConstants {
  public static final boolean kAutoDebug = false;
  public static final double kCommandMarkerThreshold = 0.05; // meters

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  // Constraint for the motion profiled robot angle controller
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  /* Auto translation constants */
  public static double kAutoXTranslationP = 2.2;
  public static double kAutoXTranslationI = 0.025;
  public static double kAutoXTranslationD = 0;

  public static double kAutoYTranslationP = 2.2;
  public static double kAutoYTranslationI = 0.025;
  public static double kAutoYTranslationD = 0;

  public static double kTranslationFF = 0.3;

  /* ThetaController constants */
  public static double kAutoThetaControllerP = 5.4;
  public static double kAutoThetaControllerI = 0.02;
  public static double kAutoThetaControllerD = 1.5;
  public static TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          AutoConstants.kMaxAngularSpeedRadiansPerSecond,
          AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

  public static final class DynamicPathGenerationConstants {
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
    private static final double emptyRow[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public static final double adjacencyGraph[][] =
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
          emptyRow,
          emptyRow
        };

    public static final PathConstraints dynamicPathConstraints = new PathConstraints(5, 5);
  }
}
