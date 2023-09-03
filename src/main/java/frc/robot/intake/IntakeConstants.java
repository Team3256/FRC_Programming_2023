// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import frc.robot.drivers.CanDeviceId;

public final class IntakeConstants {
  public static final int kIntakeMotorID = 4;
  public static final String kIntakeCANBus = "rio";
  public static final CanDeviceId kIntakeCANDevice = new CanDeviceId(kIntakeMotorID, kIntakeCANBus);
  public static int kLeftDistanceSensorID = 20;
  public static int kRightDistanceSensorID = 21;
  public static final double kIntakeWristRatio = (86.058 / 180);

  public static final double kIntakeConeSpeed = 1.0;
  public static final double kIntakeCubeSpeed = -0.85;
  public static final double kOuttakeConeSpeed = -0.45;
  public static final double kOuttakeCubeSpeed = 0.45;
  public static final double kLatchConeSpeed = 0.15;
  public static final double kLatchCubeSpeed = -0.15;

  public static final double kGamePieceMaxCurrent = 10;
  public static final double kIntakeMaxCurrent = 40;
  public static final double kTriggerThresholdTime = 0;
  public static final double kTriggerThresholdTimeCone = 0.5;
}
