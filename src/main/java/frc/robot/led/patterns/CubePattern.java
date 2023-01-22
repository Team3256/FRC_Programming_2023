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
 * Full Purple
 */
public class CubePattern extends LEDPattern{
  @Override
  public void update() {
    super.update();
    setRange(0, 99, LEDConstants.purple);
  }
}
