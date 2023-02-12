// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

import static frc.robot.led.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.util.Color;

/** Base class of LEDPattern */
public class LEDPattern {
  private Color[] pattern;

  public LEDPattern() {
    pattern = new Color[kResolution + 1];
  }

  public void updatePattern() {}

  public void setPixel(int pixel, Color color) {
    if (pixel < 1 || pixel > kResolution) return;
    pattern[pixel] = color;
  }

  public void setPixelRange(int start, int end, Color color) {
    for (int percent = start; percent <= end; percent++) {
      setPixel(percent, color);
    }
  }

  public void setPattern(LEDPattern pattern) {
    this.pattern = pattern.getPattern();
  }

  public void setPattern(Color[] pattern) {
    this.pattern = pattern;
  }

  public Color getPixel(int pixel) {
    return pattern[pixel];
  }

  public Color[] getPattern() {
    return pattern;
  }
}
