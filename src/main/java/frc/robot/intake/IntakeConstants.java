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

  public static final double kIntakeConeSpeed = 0.5;
  public static final double kIntakeCubeSpeed = -0.5;

  public final class IntakeFromDoubleSubstation {
    public static final double kArmAngle = 0.0;
    public static final double kElevatorHeight = 0.0;
  }
}
