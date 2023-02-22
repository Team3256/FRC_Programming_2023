// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AutoConstants {
  public static final boolean changeAutosBasedOnAlliance = true;
  public static final boolean kAutoDebug = false;
  public static final double kCommandMarkerThreshold = 0.05; // meters

  public static final double kMaxSpeedMetersPerSecond = 5;
  public static final double kMaxAccelerationMetersPerSecondSquared = 7.5;
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

  public static double kTranslationToleranceMeters = 0.2;
  public static double kRotationTolerance = Units.degreesToRadians(2);
  public static double kAutoTrajectoryTimeoutSeconds = 2;

  public static PathConstraints kDefaultPathConstraints =
      new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
}
