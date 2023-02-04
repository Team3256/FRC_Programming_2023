// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

import frc.robot.led.patterns.OffPattern;

public class BlinkingPattern extends AnimatedPattern {
  private final int onTicks;

  public BlinkingPattern(int onTicks, int offTicks) {
    super(onTicks + offTicks);
    this.onTicks = onTicks;
  }

  protected void setMainLEDPattern(LEDPattern ledPattern) {
    setEvent(0, ledPattern);
    setEvent(onTicks, new OffPattern());
  }
}
