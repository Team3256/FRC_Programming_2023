// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.drivers.CanDeviceId;

public final class ArmConstants {
  public static final int kArmMotorID = 6;
  public static final String kArmCanBus = "mani";
  public static final CanDeviceId kArmCANDevice = new CanDeviceId(kArmMotorID, kArmCanBus);
  public static final Rotation2d kDefaultArmAngle = Rotation2d.fromDegrees(80);
  public static final int kArmSimulationID = 16;
  public static final int kArmEncoderDIOPort = 10;
  private static final double kArmCountsPerRevolution = 8192;
  public static final double kArmEncoderConversionToRadians =
      (1 / kArmCountsPerRevolution) * 2 * Math.PI;
  public static final double kArmGearing = 240;
  public static final double kEncoderOffsetRadians = 4.2246340316;
  public static final double kArmLengthMeters = 1.638059;
  public static final double kArmInertia = 35.627712818;
  public static final double kArmMassKg = 5.10881086;
  public static final boolean kArmSimGravity = true;
  public static final int kNumArmMotors = 1;
  public static final double kArmS = 0.13794;
  public static final double kArmG = 0.60843;
  public static final double kArmV = 4.19;
  public static final double kArmA = 0.029772;
  public static final double kP = 4.4118;
  public static final double kI = 0;
  public static final double kD = 0.29266;
  public static final TrapezoidProfile.Constraints kArmContraints =
      new TrapezoidProfile.Constraints(10, 4);

  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(0.5);
  public static final Rotation2d kArmToleranceAngularVelocity = Rotation2d.fromDegrees(0.5);
  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(-12.881991);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(360);

  public static final Rotation2d kDoubleSubstationRotation = Rotation2d.fromDegrees(5);
  public static final Rotation2d kAnyPieceLowRotation = Rotation2d.fromDegrees(-30.5);
  public static final Rotation2d kCubeMidRotation = Rotation2d.fromDegrees(10);
  public static final Rotation2d kConeMidRotation = Rotation2d.fromDegrees(17.5);
  public static final Rotation2d kConeHighRotation = Rotation2d.fromDegrees(21);
  public static final Rotation2d kCubeHighRotation = Rotation2d.fromDegrees(40);
  public static final Rotation2d kGroundIntakeRotation = Rotation2d.fromDegrees(-4);
}
