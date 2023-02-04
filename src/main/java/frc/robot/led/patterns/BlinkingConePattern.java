// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.led.patternBases.BlinkingPattern;

public class BlinkingConePattern extends BlinkingPattern {

  public BlinkingConePattern() {
    super(35, 5);
    setMainLEDPattern(new ConePattern());
  }
}
