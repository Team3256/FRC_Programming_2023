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
  public static final double kElevatorStartingPosition = 0.5;
  public static final double kElevatorAngleOffset = Units.degreesToRadians(35.4);

  public static final String kElevatorCANBus = "mani";
  public static final CanDeviceId kElevatorCANDevice =
      new CanDeviceId(kElevatorMasterID, kElevatorCANBus);
  public static final CanDeviceId kElevatorFollowerCANDevice =
      new CanDeviceId(kElevatorFollowerID, kElevatorCANBus);
  public static final int kNumElevatorMotors = 2;
  public static final double kCubeStowPosition = Units.inchesToMeters(9);
  public static final double kConeStowPosition = Units.inchesToMeters(8);
  public static final double kCubeDoubleSubstationPosition = Units.inchesToMeters(47);
  public static final double kConeDoubleSubstationPosition = Units.inchesToMeters(47);
  public static final double kCubeHighPosition = Units.inchesToMeters(57);
  public static final double kConeHighPosition = Units.inchesToMeters(57);
  public static final double kAnyPieceMidPosition = Units.inchesToMeters(36);
  public static final double kAnyPieceLowPosition = Units.inchesToMeters(6);
  public static final double kGroundIntakePosition = Units.inchesToMeters(0);

  // https://www.reca.lc/linear
  public static final double kElevatorS = 0;
  public static final double kElevatorG = 0.79013;
  public static final double kElevatorV = 6.04;
  public static final double kElevatorA = 0.11;
  public static final double kElevatorP = 200;
  public static final double kElevatorI = 0;
  public static final double kElevatorD = 5;

  public static final TrapezoidProfile.Constraints kElevatorConstraints =
      new TrapezoidProfile.Constraints(5, 5);

  public static final double kDownSpeedVolts = -8.0;

  public static final double kDrumRadius = Units.inchesToMeters(1.88);
  public static final double kMinExtension = Units.inchesToMeters(0);
  public static final double kMaxExtension = Units.inchesToMeters(57);
  public static final double kElevatorGearing = 15.5;
  public static final double kCarriageMass = 9; // kg
  public static final double kTolerancePosition = Units.inchesToMeters(1);
  public static final double kToleranceVelocity = Units.inchesToMeters(1);
  public static final double kRateLimiting = 0.05;
  public static final double kElevatorCurrentThreshold = 30; // amps

  public static class ElevatorPreferencesKeys {
    public static final Map<Elevator.ElevatorPreset, String> kElevatorPositionKeys =
        Map.of(
            Elevator.ElevatorPreset.STOW_CONE, "kStowPositionCone",
            Elevator.ElevatorPreset.STOW_CUBE, "kStowPositionCube",
            Elevator.ElevatorPreset.CUBE_HIGH, "kCubeHighPositionMeters",
            Elevator.ElevatorPreset.CONE_HIGH, "kConeHighPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_LOW, "kAnyPieceLowPositionMeters",
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
            Elevator.ElevatorPreset.ANY_PIECE_LOW, kAnyPieceLowPosition,
            Elevator.ElevatorPreset.ANY_PIECE_MID, kAnyPieceMidPosition,
            Elevator.ElevatorPreset.GROUND_INTAKE, kGroundIntakePosition,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE, kConeDoubleSubstationPosition,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE, kCubeDoubleSubstationPosition);

    public static final String kPKey = "ElevatorkP";
    public static final String kIKey = "ElevatorkI";
    public static final String kDKey = "ElevatorkD";
  }
}
