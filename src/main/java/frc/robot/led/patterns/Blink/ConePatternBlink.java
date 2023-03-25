// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns.Blink;

import frc.robot.led.patternBases.BlinkingPattern;
import frc.robot.led.patterns.ConePattern;

/** Blinking Yellow */
public class ConePatternBlink extends BlinkingPattern {

  public ConePatternBlink() {
    super(40, 5);
    setMainLEDPattern(new ConePattern());
  }
}
