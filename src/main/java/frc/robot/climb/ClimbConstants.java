// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.climb;

import frc.robot.drivers.CanDeviceId;

public final class ClimbConstants {
  // TODO: Check CAN ID and tune position before merging

  public static final int kClimbMotorID = 7;
  public static final String kClimbCANBus = "mani";

  public static final CanDeviceId kClimbCANDevice = new CanDeviceId(kClimbMotorID, kClimbCANBus);

  public static final double kClimbGearing = 125;
  public static final double kClimbEncoderTicksPerRotation = 2048;

  public static final double kClimbDeployPosition =
      1.68 * kClimbEncoderTicksPerRotation * kClimbGearing;
  public static final double kClimbRetractPosition = kClimbDeployPosition * 2;
}
