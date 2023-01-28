// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

import frc.robot.drivers.Color;

/** Base class of LEDPattern */
public class LEDPattern {
  private Color[] pattern;

  public LEDPattern() {
    pattern = new Color[101];
    for (int i = 1; i <= 100; i++) pattern[i] = new Color();
  }

  public void update() {}

  protected void set(int percent, Color color) {
    pattern[percent].set(color);
  }

  protected void setRange(int start, int end, Color color) {
    for (int percent = start; percent <= end; percent++) {
      set(percent, color);
    }
  }

  protected void setPattern(Color[] pattern) {
    this.pattern = pattern;
  }

  protected Color get(int percent) {
    return pattern[percent];
  }

  public Color[] getPattern() {
    return pattern;
  }
}
