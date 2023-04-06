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
    public static final Map<Arm.ArmPreset, String> kArmPositionKeys =
        Map.of(
            Arm.ArmPreset.DEFAULT, "kDefaultArmRotation",
            Arm.ArmPreset.ANY_PIECE_LOW, "kAnyPieceLowRotation",
            Arm.ArmPreset.CUBE_MID, "kCubeMidRotation",
            Arm.ArmPreset.CONE_MID, "kConeMidRotation",
            Arm.ArmPreset.CUBE_HIGH, "kCubeHighRotation",
            Arm.ArmPreset.CONE_HIGH, "kConeHighRotation",
            Arm.ArmPreset.GROUND_INTAKE, "kGroundIntakeRotation",
            Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, "kDoubleSubstationCubeRotation",
            Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, "kDoubleSubstationConeRotation");

    public static final Map<Arm.ArmPreset, Rotation2d> kArmPositionDefaults =
        Map.of(
            Arm.ArmPreset.DEFAULT, kDefaultArmAngle,
            Arm.ArmPreset.ANY_PIECE_LOW, kAnyPieceLowRotation,
            Arm.ArmPreset.CUBE_MID, kCubeMidRotation,
            Arm.ArmPreset.CONE_MID, kConeMidRotation,
            Arm.ArmPreset.CUBE_HIGH, kCubeHighRotation,
            Arm.ArmPreset.CONE_HIGH, kConeHighRotation,
            Arm.ArmPreset.GROUND_INTAKE, kGroundIntakeRotation,
            Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, kDoubleSubstationRotationCone,
            Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, kDoubleSubstationRotationCube);

    public static final String kPKey = "ArmkP";
    public static final String kIKey = "ArmkI";
    public static final String kDKey = "ArmkD";
    public static final String kRelativeEncoderOffsetKey = "kRelativeEncoderOffset";
    public static final String kAbsoluteEncoderOffsetKey = "kAbsoluteEncoderOffset";
  }

  public static final int kArmMotorID = 6;
  public static final String kArmCanBus = "mani";
  public static final CanDeviceId kArmCANDevice = new CanDeviceId(kArmMotorID, kArmCanBus);
  public static final int kArmSimulationID = 16;
  public static final int kArmEncoderDIOPort = 8;

  public static final double kArmRadiansPerAbsoluteEncoderRotation = 2 * Math.PI;
  public static final double kAbsoluteEncoderOffsetRadians = -1.666789;

  public static final double kArmGearing = 240;
  public static double kRelativeFalconEncoderOffsetRadians =
      Constants.kCompetitionModeEnabled ? 4.2246340316 : Math.PI / 2;
  public static final double kArmLengthMeters = 1.638059;
  public static final double kArmInertia = 35.627712818;
  public static final double kArmMassKg = 5.10881086;
  public static final boolean kArmSimGravity = true;
  public static final int kNumArmMotors = 1;

  public static final double kArmS = 0.16924;
  public static final double kArmG = 0.15542;
  public static final double kArmV = 4.1445;
  public static final double kArmA = 0.17676;
  public static final double kP = 10.96;
  public static final double kI = 0;
  public static final double kD = 7.0262;

  //  CONSTANTS FOR NO GAS SHOCK BELOW:

  //  public static final double kArmS = 0.91487;
  //  public static final double kArmG = 0.98236;
  //  public static final double kArmV = 2.9364;
  //  public static final double kArmA = 0.18338;
  //  public static final double kP = 10.608;
  //  public static final double kI = 0;
  //  public static final double kD = 7.8777;

  public static final TrapezoidProfile.Constraints kArmProfileContraints =
      new TrapezoidProfile.Constraints(5, 2);
  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(0.5);
  public static final Rotation2d kArmToleranceAngularVelocity = Rotation2d.fromDegrees(0.5);

  // TODO Tune later
  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(-35);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(180);

  public static final Rotation2d kDefaultArmAngle = Rotation2d.fromDegrees(90);
  public static final Rotation2d kDoubleSubstationRotationCube = new Rotation2d(0.07);
  public static final Rotation2d kDoubleSubstationRotationCone = new Rotation2d(0.01);
  public static final Rotation2d kAnyPieceLowRotation = Rotation2d.fromDegrees(-30.5);
  public static final Rotation2d kCubeMidRotation = new Rotation2d(0.32);
  public static final Rotation2d kConeMidRotation = new Rotation2d(0.18);
  public static final Rotation2d kCubeHighRotation = new Rotation2d(0.6);
  public static final Rotation2d kConeHighRotation = new Rotation2d(0.23);
  public static final Rotation2d kGroundIntakeRotation = Rotation2d.fromDegrees(-10);

  public static final double kManualArmVoltage = 2.5;
}
