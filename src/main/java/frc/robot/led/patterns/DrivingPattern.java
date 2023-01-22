// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.Constants.LEDConstants;
import frc.robot.led.patternBases.LEDPattern;

/**
 * Half red half empty
 */
public class DrivingPattern extends LEDPattern{
  @Override
  public void update() {
    super.update();
    setRange(0, 49,LEDConstants.red);
    setRange(50,99,LEDConstants.off);
  }
}
