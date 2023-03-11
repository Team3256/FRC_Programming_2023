// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import edu.wpi.first.math.util.Units;

public class LEDConstants {
  public static final int kResolution = 100;
  public static final int kLEDPWMPort = 9;
  public static final int kNumberOfLEDs = 55;
  public static final double kInterval = Units.secondsToMilliseconds(0.1);
  public static final double kIntervalTrail =
                Units.secondsToMilliseconds(0.1);
}
