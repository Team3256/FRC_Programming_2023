// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns.Blink;

import frc.robot.led.patternBases.BlinkingPattern;
import frc.robot.led.patterns.LockSwervePattern;

public class LockSwerveLEDBlink extends BlinkingPattern {

  public LockSwerveLEDBlink() {
    super(20, 4);
    setMainLEDPattern(new LockSwervePattern());
  }
}
