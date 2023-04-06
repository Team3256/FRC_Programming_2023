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
  public static final double kElevatorStartingPositionMeters = 0.5;
  public static final String kElevatorCANBus = "mani";
  public static final CanDeviceId kElevatorCANDevice =
      new CanDeviceId(kElevatorMasterID, kElevatorCANBus);
  public static final CanDeviceId kElevatorFollowerCANDevice =
      new CanDeviceId(kElevatorFollowerID, kElevatorCANBus);
  public static final int kNumElevatorMotors = 1;
  public static final boolean kElevatorInverted = true;

  public static final double kZeroThreshold = 0.02;

  public static final double kDoubleSubstationPositionCubeMeters = 0.455;
  public static final double kDoubleSubstationPositionConeMeters = 0.55;
  public static final double kCubeHighPositionMeters = 0.2;
  public static final double kConeHighPositionMeters = 0.43;
  public static final double kAnyPieceMidPositionMeters = 0.16;
  public static final double kAnyPieceLowPositionMeters = Units.inchesToMeters(31);
  public static final double kGroundIntakePositionMeters = Units.inchesToMeters(0);

  public static class ElevatorPreferencesKeys {
    public static final Map<Elevator.ElevatorPreset, String> kElevatorPositionKeys =
        Map.of(
            Elevator.ElevatorPreset.CUBE_HIGH, "kCubeHighPositionMeters",
            Elevator.ElevatorPreset.CONE_HIGH, "kConeHighPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_LOW, "kAnyPieceLowPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_MID, "kAnyPieceMidPositionMeters",
            Elevator.ElevatorPreset.GROUND_INTAKE, "kGroundIntakePositionMeters",
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE, "kDoubleSubstationPositionConeMeters",
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE, "kDoubleSubstationPositionCubeMeters");

    public static final Map<Elevator.ElevatorPreset, Double> kElevatorPositionDefaults =
        Map.of(
            Elevator.ElevatorPreset.CUBE_HIGH, kCubeHighPositionMeters,
            Elevator.ElevatorPreset.CONE_HIGH, kConeHighPositionMeters,
            Elevator.ElevatorPreset.ANY_PIECE_LOW, kAnyPieceLowPositionMeters,
            Elevator.ElevatorPreset.ANY_PIECE_MID, kAnyPieceMidPositionMeters,
            Elevator.ElevatorPreset.GROUND_INTAKE, kGroundIntakePositionMeters,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE, kDoubleSubstationPositionConeMeters,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE, kDoubleSubstationPositionCubeMeters);

    public static final String kPKey = "ElevatorkP";
    public static final String kIKey = "ElevatorkI";
    public static final String kDKey = "ElevatorkD";
  }

  // https://www.reca.lc/linear
  // gradle simulateJava working constants
  // for some reason the elevator acts differntly in unit tests vs sim
  public static final double kElevatorS = 0.22563;
  public static final double kElevatorG = 0.76032;
  public static final double kElevatorV = 9.19241;
  public static final double kElevatorA = 0;

  // public static final double kP = 0.0032534;
  public static final double kP = 18;
  public static final double kI = 0;
  public static final double kD = 0;
  // public static final double kD = 0.0012892;

  public static final TrapezoidProfile.Constraints kElevatorContraints =
      new TrapezoidProfile.Constraints(2.45, 2.45);

  public static final double kDownSpeedVolts = -8.0;

  public static final double kDrumRadius = 0.0222377;
  public static final double kMinHeight = Units.inchesToMeters(10);
  public static final double kMaxHeight = Units.inchesToMeters(30);
  public static final double kElevatorGearing = 15;
  public static final double kCarriageMass = 6.28815086; // kg

  public static final double kTolerancePosition = Units.inchesToMeters(1);
  public static final double kToleranceVelocity = Units.inchesToMeters(1);
  public static final double kRateLimiting = 0.05;
  public static final double kElevatorCurrentThreshold = 30; // amps
}
