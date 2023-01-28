// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.Constants.LEDConstants;
import frc.robot.led.patternBases.AnimatedPattern;
import frc.robot.led.patternBases.LEDPattern;

public class SpiritPattern extends AnimatedPattern {
  public SpiritPattern() {
    super(100);
    LEDPattern blue = new LEDPattern();
    blue.setRange(1, 100, LEDConstants.kBlue);
    LEDPattern white = new LEDPattern();
    white.setRange(1, 100, LEDConstants.kWhite);
    setEvent(0, blue);
    setEvent(50, white);
  }
}
