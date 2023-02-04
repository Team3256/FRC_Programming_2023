// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.Constants.LEDConstants;
import frc.robot.led.patternBases.BlinkingPattern;
import frc.robot.led.patternBases.LEDPattern;

/** Blinking Half red half empty */
public class DrivingPattern extends BlinkingPattern {
  public DrivingPattern() {
    super(50, 50);
    LEDPattern main = new LEDPattern();
    main.setRangeOfPixels(1, 25, LEDConstants.kRed);
    main.setRangeOfPixels(26, 50, LEDConstants.kOff);
    main.setRangeOfPixels(51, 75, LEDConstants.kRed);
    main.setRangeOfPixels(76, 100, LEDConstants.kOff);
    setMainPattern(main.getPattern());
  }
}
