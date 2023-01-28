// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

import frc.robot.led.patterns.OffPattern;

public class BlinkingPattern extends AnimatedPattern {
  private int onTicks;

  public BlinkingPattern(int onTicks, int offTicks) {
    super(onTicks + offTicks);
    this.onTicks = onTicks;
  }

  protected void setMainPattern(LEDPattern pattern) {
    setEvent(0, pattern);
    setEvent(onTicks, new OffPattern());
  }
}
