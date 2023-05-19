// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.drivers.CanDeviceId;
import java.util.Map;

public final class ElevatorConstants {
  public static final int kElevatorMasterID = 5;
  public static final int kElevatorFollowerID = 14;
  public static final int kElevatorLimitSwitchDIO = 9;
  public static final double kElevatorStartingPosition = 0.5;
  public static final double kElevatorAngleOffset = Units.degreesToRadians(35.4);
  public static final String kElevatorCANBus = "rio";
  public static final CanDeviceId kElevatorCANDevice =
      new CanDeviceId(kElevatorMasterID, kElevatorCANBus);
  public static final CanDeviceId kElevatorFollowerCANDevice =
      new CanDeviceId(kElevatorFollowerID, kElevatorCANBus);
  public static final int kNumElevatorMotors = 2;
  public static final double kCubeStowPosition = Units.inchesToMeters(4);
  public static final double kConeStowPosition = Units.inchesToMeters(4);
  public static final double kCubeDoubleSubstationPosition = Units.inchesToMeters(46);
  public static final double kConeDoubleSubstationPosition = Units.inchesToMeters(46);
  public static final double kCubeHighPosition = Units.inchesToMeters(57);
  public static final double kConeHighPosition = Units.inchesToMeters(57);
  public static final double kAnyPieceMidPosition = Units.inchesToMeters(37);
  public static final double kAnyPieceLowBackPosition = Units.inchesToMeters(0);
  public static final double kAnyPieceLowFrontPosition = Units.inchesToMeters(15);
  public static final double kGroundIntakePosition = Units.inchesToMeters(0);
  public static final double kSafeForArmMinPosition =
      kAnyPieceLowFrontPosition - Units.inchesToMeters(7);

  public static final double kElevatorS = 0.45;
  public static final double kElevatorV = 7.30;
  public static final double kElevatorA = 0.01;
  public static final double kElevatorG = 0.00;
  public static final double kElevatorP = 30;
  public static final double kElevatorI = 0;
  public static final double kElevatorD = 0;

  public static final TrapezoidProfile.Constraints kElevatorConstraints =
      new TrapezoidProfile.Constraints(3.5, 2.00);

  public static final double kDownSpeedVolts = -3;
  public static final double kElevatorCurrentThreshold = 64; // amps

  public static final double kDrumRadius = Units.inchesToMeters(0.94);
  public static final double kMinExtension = Units.inchesToMeters(0);
  public static final double kMaxExtension = Units.inchesToMeters(59);
  public static final double kElevatorGearing = 527 / 45;
  public static final double kCarriageMass = 9; // kg
  public static final double kTolerancePosition = Units.inchesToMeters(2.5);
  public static final double kToleranceVelocity = Units.inchesToMeters(2.5);
  public static final double kRateLimiting = 0.05;

  public static class ElevatorPreferencesKeys {
    public static final Map<Elevator.ElevatorPreset, String> kElevatorPositionKeys =
        Map.of(
            Elevator.ElevatorPreset.STOW_CONE, "kStowPositionCone",
            Elevator.ElevatorPreset.STOW_CUBE, "kStowPositionCube",
            Elevator.ElevatorPreset.CUBE_HIGH, "kCubeHighPositionMeters",
            Elevator.ElevatorPreset.CONE_HIGH, "kConeHighPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_LOW_BACK, "kAnyPieceBackLowPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_LOW_FRONT, "kAnyPieceFrontLowPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_MID, "kAnyPieceMidPositionMeters",
            Elevator.ElevatorPreset.GROUND_INTAKE, "kGroundIntakePositionMeters",
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE, "kDoubleSubstationPositionConeMeters",
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE, "kDoubleSubstationPositionCubeMeters");

    public static final Map<Elevator.ElevatorPreset, Double> kElevatorPositionDefaults =
        Map.of(
            Elevator.ElevatorPreset.STOW_CONE, kConeStowPosition,
            Elevator.ElevatorPreset.STOW_CUBE, kCubeStowPosition,
            Elevator.ElevatorPreset.CUBE_HIGH, kCubeHighPosition,
            Elevator.ElevatorPreset.CONE_HIGH, kConeHighPosition,
            Elevator.ElevatorPreset.ANY_PIECE_LOW_BACK, kAnyPieceLowBackPosition,
            Elevator.ElevatorPreset.ANY_PIECE_MID, kAnyPieceMidPosition,
            Elevator.ElevatorPreset.GROUND_INTAKE, kGroundIntakePosition,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE, kConeDoubleSubstationPosition,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE, kCubeDoubleSubstationPosition);

    public static final String kPKey = "ElevatorkP";
    public static final String kIKey = "ElevatorkI";
    public static final String kDKey = "ElevatorkD";
  }
}
