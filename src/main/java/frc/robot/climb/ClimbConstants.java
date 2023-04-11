// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.climb;

import frc.robot.drivers.CanDeviceId;

public final class ClimbConstants {
  public static final int kClimbMotorID = 7;
  public static final String kClimbCANBus = "mani";

  public static final CanDeviceId kClimbCANDevice = new CanDeviceId(kClimbMotorID, kClimbCANBus);

  public static final double kClimbGearing = 125;
  public static final double kClimbEncoderTicksPerRotation = 2048;

  public static class ClimbPreferencesKeys {
    public static final String kClimbDeployRotationKey = "kClimbDeployRotation";
    public static final String kClimbRetractRotationKey = "kClimbRetractRotation";
    public static final String kClimbDeploySpeedKey = "kClimbDeploySpeed";
    public static final String kClimbRetractSpeedKey = "kClimbRetractSpeed";
  }

  public static final double kClimbDeployRotation = 1.68;
  public static final double kClimbRetractRotation = kClimbDeployRotation * 2;
  public static final double kClimbDeploySpeed = 0.9;
  public static final double kClimbRetractSpeed = -1.0;
}
