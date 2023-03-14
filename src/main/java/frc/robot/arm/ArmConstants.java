// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.drivers.CanDeviceId;
import java.util.Map;

public final class ArmConstants {
  public static class ArmPreferencesKeys {
    public static final Map<Arm.ArmPosition, String> kArmPositionKeys =
        Map.of(
            Arm.ArmPosition.DEFAULT, "kDefaultArmAngle",
            Arm.ArmPosition.ANY_PIECE_LOW, "kAnyPieceLowRotation",
            Arm.ArmPosition.CUBE_MID, "kCubeMidRotation",
            Arm.ArmPosition.CONE_MID, "kConeMidRotation",
            Arm.ArmPosition.CUBE_HIGH, "kCubeHighRotation",
            Arm.ArmPosition.CONE_HIGH, "kConeHighRotation",
            Arm.ArmPosition.GROUND_INTAKE, "kGroundIntakeRotation",
            Arm.ArmPosition.DOUBLE_SUBSTATION, "kDoubleSubstationRotation");

    public static final Map<Arm.ArmPosition, Rotation2d> kArmPositionDefaults =
        Map.of(
            Arm.ArmPosition.DEFAULT, kDefaultArmAngle,
            Arm.ArmPosition.ANY_PIECE_LOW, kAnyPieceLowRotation,
            Arm.ArmPosition.CUBE_MID, kCubeMidRotation,
            Arm.ArmPosition.CONE_MID, kConeMidRotation,
            Arm.ArmPosition.CUBE_HIGH, kCubeHighRotation,
            Arm.ArmPosition.CONE_HIGH, kConeHighRotation,
            Arm.ArmPosition.GROUND_INTAKE, kGroundIntakeRotation,
            Arm.ArmPosition.DOUBLE_SUBSTATION, kDoubleSubstationRotation);

    public static final String kPKey = "ArmkP";
    public static final String kIKey = "ArmkI";
    public static final String kDKey = "ArmkD";
  }

  // TODO: Fix constants
  public static final int kArmMotorID = 6;
  public static final String kArmCanBus = "mani";
  public static final CanDeviceId kArmCANDevice = new CanDeviceId(kArmMotorID, kArmCanBus);
  public static final int kArmSimulationID = 16;
  public static final int kArmEncoderDIOPort = 10;
  private static final double kArmCountsPerRevolution = 8192;
  public static final double kArmEncoderConversionToRadians =
      (1 / kArmCountsPerRevolution) * 2 * Math.PI;

  public static final double kArmGearing = 240;
  public static double kEncoderOffsetRadians =
      Constants.kCompetitionModeEnabled ? 4.2246340316 : Math.PI / 2;
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

  public static final TrapezoidProfile.Constraints kArmProfileContraints =
      new TrapezoidProfile.Constraints(8, 4);
  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(0.5);
  public static final Rotation2d kArmToleranceAngularVelocity = Rotation2d.fromDegrees(0.5);

  // TODO Tune later
  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(-35);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(150);

  public static final Rotation2d kDefaultArmAngle =
      Constants.kCompetitionModeEnabled ? Rotation2d.fromDegrees(72) : Rotation2d.fromDegrees(90);
  public static final Rotation2d kDoubleSubstationRotation = Rotation2d.fromDegrees(5.5);
  public static final Rotation2d kAnyPieceLowRotation = Rotation2d.fromDegrees(-30.5);
  public static final Rotation2d kCubeMidRotation = Rotation2d.fromDegrees(12);
  public static final Rotation2d kConeMidRotation = Rotation2d.fromDegrees(17.5);
  public static final Rotation2d kConeHighRotation = Rotation2d.fromDegrees(21);
  public static final Rotation2d kCubeHighRotation = Rotation2d.fromDegrees(40);
  public static final Rotation2d kGroundIntakeRotation = Rotation2d.fromDegrees(-10);

  public static final double kManualArmVoltage = 3.5;
}
