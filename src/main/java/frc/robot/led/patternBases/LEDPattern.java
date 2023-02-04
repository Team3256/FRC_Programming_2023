// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

import frc.robot.Constants.LEDConstants;
import frc.robot.drivers.Color;

/** Base class of LEDPattern */
public class LEDPattern {
  private Color[] pattern;

  public LEDPattern() {
    pattern = new Color[LEDConstants.kResolution + 1];
    for (int i = 1; i <= LEDConstants.kResolution; i++) pattern[i] = new Color();
  }

  public void update() {}

  // getters and setters
  public void set(int percent, Color color) {
    if (percent < 1 || percent > LEDConstants.kResolution) return;
    pattern[percent].set(color);
  }

  public void setRange(int start, int end, Color color) {
    for (int percent = start; percent <= end; percent++) {
      set(percent, color);
    }
  }

  public void setPattern(Color[] pattern) {
    this.pattern = pattern;
  }

  public void setPattern(LEDPattern ledPattern) {
    this.pattern = ledPattern.getPattern();
  }

  public Color get(int percent) {
    return pattern[percent];
  }

  public Color[] getPattern() {
    return pattern;
  }
}
