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
  public static final String kIntakeCANBus = "mani";
  public static final CanDeviceId kIntakeCANDevice = new CanDeviceId(kIntakeMotorID, kIntakeCANBus);

  public static final double kIntakeConeSpeed = 0.9;
  public static final double kIntakeCubeSpeed = -0.9;
  public static final double kLatchConeSpeed = 0.2;
  public static final double kLatchCubeSpeed = -0.2;

  public static final double kConeMaxCurrent = 12;
  public static final double kCubeMaxCurrent = 10;
  public static final double kIntakeMaxCurrent = 40;

  public static final double kIntakeFinishedMaxError = 0.1;
  public static final double kIntakeCurrentTriggerThresholdTime = 0.2;
}
