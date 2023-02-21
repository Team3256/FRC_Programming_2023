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

public final class ElevatorConstants {
  public static final int kElevatorID = 5;
  public static final String kElevatorCANBus = "mani";
  public static final CanDeviceId kElevatorCANDevice =
      new CanDeviceId(kElevatorID, kElevatorCANBus);
  public static final int kNumElevatorMotors = 1;
  public static final boolean kElevatorInverted = true;

  // TODO: Change to real values
  public static final double kElevatorHighPositionMeters = Units.inchesToMeters(30);
  public static final double kElevatorMidPositionMeters = Units.inchesToMeters(15);
  public static final double kElevatorLowPositionMeters = Units.inchesToMeters(3);

  // https://www.reca.lc/linear
  // gradle simulateJava working constants
  // for some reason the elevator acts differntly in unit tests vs sim
  public static final double kElevatorS = 0;
  public static final double kElevatorG = 1.62;
  public static final double kElevatorV = 1.31;
  public static final double kElevatorA = 0.13;

  public static final double kP = 1.1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final TrapezoidProfile.Constraints kElevatorContraints =
      new TrapezoidProfile.Constraints(2.45, 2.45);

  public static final double kDownSpeedVolts = -2.0;

  public static final double kDrumRadius = 0.0222377;
  public static final double kMinHeight = 0.174625;
  public static final double kMaxHeight = 1.09855;
  public static final double kElevatorGearing = 5;
  public static final double kCarriageMass = 6.28815086; // kg

  public static final double kTolerancePosition = 0.025;
  public static final double kToleranceVelocity = 0.025;
  // TODO: TUNE THIS
  public static final double kRateLimiting = 0.05;
  public static final double kEncoderToMetersConversionFactor = 1;
  public static final double kElevatorCurrentThreshold = 8; // amps
}
