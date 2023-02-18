// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class ArmConstants {
  // TODO: Fix constants
  public static final int kArmMotorID = 20;
  public static final int kArmEncoderDIOPort = 10;

  private static final double kArmCountsPerRevolution = 8192;
  public static final double kArmEncoderConversionToRadians = (1 / kArmCountsPerRevolution) * 2 * Math.PI;

  // TODO Update
  public static final double kArmLengthMeters = 1.638059;
  public static final double kArmInertia = 30;
  public static final double kArmMassKg = 5.10881086;
  public static final double kArmGearing = 240;
  public static final boolean kArmSimGravity = true;

  public static final double kP = 10.0;
  public static final double kI = 0;
  public static final double kD = 0.5;
  public static final TrapezoidProfile.Constraints kArmContraints = new TrapezoidProfile.Constraints(2, 1);

  // TODO Update
  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d kArmToleranceAngularVelocity = Rotation2d.fromDegrees(0);
  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(-12.881991);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(90);

  // https://www.reca.lc/arm
  // TODO Update
  public static final double kArmS = 0.0;
  public static final double kArmG = 5.21;
  public static final double kArmV = 0.6;
  public static final double kArmA = 1.81;
  public static final int kNumArmMotors = 1;
}
