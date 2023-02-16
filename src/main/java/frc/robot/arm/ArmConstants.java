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
  public static final int kArmMotorID = 10;
  public static final int kArmEncoderDIOPort = 1;

  private static final double kArmCountsPerRevolution = 256;
  public static final double kArmEncoderConversionToRadians =
      (1 / kArmCountsPerRevolution) * 2 * Math.PI;

  // TODO Update
  public static final double kArmLengthMeters = 3.5;
  public static final double kArmInertia = 10;
  public static final double kArmMassKg = 5.5;
  public static final double kArmGearing = 100;
  public static final boolean kArmSimGravity = true;

  public static final double kP = 100;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final TrapezoidProfile.Constraints kArmContraints =
      new TrapezoidProfile.Constraints(2, 1);

  // TODO Update
  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d kArmToleranceAngularVelocity = Rotation2d.fromDegrees(0);
  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(-12.881991);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(90);

  // https://www.reca.lc/arm
  // TODO Update
  public static final double kArmS = 0.5;
  public static final double kArmG = 4.59;
  public static final double kArmV = 1.8;
  public static final double kArmA = 1.81;
  public static final int kNumArmMotors = 1;
}
