// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import static frc.robot.led.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.patternBases.LEDPattern;

/** Full Green */
public class DoubleSubstationDonePattern extends LEDPattern {
  public DoubleSubstationDonePattern() {
    super();
    setPixelRange(1, kResolution, Color.kDarkGreen);
  }
}
