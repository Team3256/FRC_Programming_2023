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
import frc.robot.Constants;

public final class AutoConstants {
  public static final boolean kAutoDebug = false && Constants.kDebugEnabled;
  public static final boolean changeAutosBasedOnAlliance = true;
  public static final double kCommandMarkerThreshold = 0.05; // meters

  public static final double kMaxSpeedMetersPerSecond = 5;
  public static final double kMaxAccelerationMetersPerSecondSquared = 7.5;
  public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

  // Constraint for the motion profiled robot angle controller
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  /* Auto translation constants */
  public static final double kAutoXTranslationP = 2.2;
  public static final double kAutoXTranslationI = 0.025;
  public static final double kAutoXTranslationD = 0;

  public static final double kAutoYTranslationP = 2.2;
  public static final double kAutoYTranslationI = 0.025;
  public static final double kAutoYTranslationD = 0;

  public static final double kTranslationFF = 0.3;

  /* ThetaController constants */
  public static final double kAutoThetaControllerP = 6.0;
  public static final double kAutoThetaControllerI = 0.00;
  public static final double kAutoThetaControllerD = 0.0;
  public static final TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          AutoConstants.kMaxAngularSpeedRadiansPerSecond,
          AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

  public static final double kTranslationToleranceMeters = 0.5;
  public static final double kAutoMarkerTimeThreshold = 0.1; // seconds
  public static final double kAutoMarkerTimeout = 0.5; // seconds
  public static final double kRotationTolerance = Units.degreesToRadians(5);
  public static final double kAutoTrajectoryTimeoutSeconds = 0.1;

  public static final PathConstraints kFastPathConstraints =
      new PathConstraints(7, kMaxAccelerationMetersPerSecondSquared);
  public static final PathConstraints kSafePathConstraints = new PathConstraints(5, 5);
  public static final PathConstraints kEngagePathConstraints = new PathConstraints(3, 3);
  public static final PathConstraints kGroundIntakeConstraints = new PathConstraints(1.5, 1.5);
}
