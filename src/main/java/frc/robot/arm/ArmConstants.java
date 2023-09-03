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
import java.util.Map;

public final class ArmConstants {
  public static final int kArmMotorID = 6;
  public static final String kArmCanBus = "rio";
  public static final CanDeviceId kArmCANDevice = new CanDeviceId(kArmMotorID, kArmCanBus);
  public static final int kArmSimulationID = 16;
  public static final int kArmEncoderDIOPort = 8;

  public static final double kArmEncoderGearRatio = 2 / 1;
  public static final double kArmRadiansPerAbsoluteEncoderRotation =
      -(2 / kArmEncoderGearRatio) * Math.PI;
  public static final double kAbsoluteEncoderOffsetRadians = 3.6506685;

  public static final double kArmGearing = 80;
  public static final double kArmLength = 0.569075;
  public static final double kArmInertia = 0.410;
  public static final double kArmMassKg = 7.5;
  public static final int kNumArmMotors = 1;

  public static final double kArmS = 0.21305;
  public static final double kArmV = 0.23;
  public static final double kArmA = 0.011292;
  public static final double kArmG = 0.72163;
  public static final double kArmP = 6.5;
  public static final double kArmI = 0;
  public static final double kArmD = 0.25699;

  public static final TrapezoidProfile.Constraints kArmProfileContraints =
      new TrapezoidProfile.Constraints(16, 16);

  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(5);
  public static final Rotation2d kArmToleranceAngularVelocity = Rotation2d.fromDegrees(5);

  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(0);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(180);

  public static final Rotation2d kStowRotationCube = Rotation2d.fromDegrees(35);
  public static final Rotation2d kStowRotationCone = Rotation2d.fromDegrees(35);

  public static final Rotation2d kDoubleSubstationRotationCube = new Rotation2d(0);
  public static final Rotation2d kDoubleSubstationRotationCone = new Rotation2d(0);
  public static final Rotation2d kCubeMidRotation = new Rotation2d(0);
  public static final Rotation2d kConeMidRotation = new Rotation2d(0);
  public static final Rotation2d kCubeHighRotation = new Rotation2d(0);
  public static final Rotation2d kConeHighRotation = new Rotation2d(0);
  public static final Rotation2d kAnyPieceLowBackRotation = Rotation2d.fromDegrees(135);
  public static final Rotation2d kAnyPieceLowFrontRotation = Rotation2d.fromDegrees(0);

  public static final Rotation2d kStandingConeGroundIntakeRotation =
      Rotation2d.fromDegrees(142.644);
  public static final Rotation2d kTippedConeGroundIntakeRotation = Rotation2d.fromDegrees(180);
  public static final Rotation2d kCubeGroundIntakeRotation = Rotation2d.fromDegrees(180);
  public static final double kManualArmVoltage = 2.5;
  public static final double kMinSafeRotation = 15;
  public static final double kMaxSafeRotation = 130;

  public static class ArmPreferencesKeys {
    public static final Map<Arm.ArmPreset, String> kArmPositionKeys =
        Map.ofEntries(
            Map.entry(Arm.ArmPreset.STOW_CONE, "kStowRotationCone"),
            Map.entry(Arm.ArmPreset.STOW_CUBE, "kStowRotationCube"),
            Map.entry(Arm.ArmPreset.ANY_PIECE_LOW_BACK, "kAnyPieceLowBackRotation"),
            Map.entry(Arm.ArmPreset.ANY_PIECE_LOW_FRONT, "kAnyPieceLowFrontRotation"),
            Map.entry(Arm.ArmPreset.CUBE_MID, "kCubeMidRotation"),
            Map.entry(Arm.ArmPreset.CONE_MID, "kConeMidRotation"),
            Map.entry(Arm.ArmPreset.CUBE_HIGH, "kCubeHighRotation"),
            Map.entry(Arm.ArmPreset.CONE_HIGH, "kConeHighRotation"),
            Map.entry(Arm.ArmPreset.CUBE_GROUND_INTAKE, "kCubeGroundIntakeRotation"),
            Map.entry(
                Arm.ArmPreset.STANDING_CONE_GROUND_INTAKE, "kStandingConeGroundIntakeRotation"),
            Map.entry(Arm.ArmPreset.SITTING_CONE_GROUND_INTAKE, "kTippedConeGroundIntakeRotation"),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, "kDoubleSubstationCubeRotation"),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, "kDoubleSubstationConeRotation"));

    public static final Map<Arm.ArmPreset, Rotation2d> kArmPositionDefaults =
        Map.ofEntries(
            Map.entry(Arm.ArmPreset.STOW_CONE, kStowRotationCone),
            Map.entry(Arm.ArmPreset.STOW_CUBE, kStowRotationCube),
            Map.entry(Arm.ArmPreset.ANY_PIECE_LOW_BACK, kAnyPieceLowBackRotation),
            Map.entry(Arm.ArmPreset.CUBE_MID, kCubeMidRotation),
            Map.entry(Arm.ArmPreset.CONE_MID, kConeMidRotation),
            Map.entry(Arm.ArmPreset.CUBE_HIGH, kCubeHighRotation),
            Map.entry(Arm.ArmPreset.CONE_HIGH, kConeHighRotation),
            Map.entry(Arm.ArmPreset.CUBE_GROUND_INTAKE, kCubeGroundIntakeRotation),
            Map.entry(Arm.ArmPreset.STANDING_CONE_GROUND_INTAKE, kStandingConeGroundIntakeRotation),
            Map.entry(Arm.ArmPreset.SITTING_CONE_GROUND_INTAKE, kTippedConeGroundIntakeRotation),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, kDoubleSubstationRotationCube),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, kDoubleSubstationRotationCone));

    public static final String kPKey = "ArmkP";
    public static final String kIKey = "ArmkI";
    public static final String kDKey = "ArmkD";
    public static final String kAbsoluteEncoderOffsetKey = "kAbsoluteEncoderOffset";
  }

  // Zero Arm
  public static final double kZeroArmVoltage = -6.0;
  public static final double kZeroArmCurrentThreshold = 50.0;
}
