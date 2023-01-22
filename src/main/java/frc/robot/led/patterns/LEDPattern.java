// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.drivers.Color;

/**
 * Only extend this class
 * The foundation of a displayable and
 * periodically updating LEDPattern
 */
public class LEDPattern {
  public int startPercentage;
  public int endPercentage;
  public int length;
  public Color[] totalPattern;

  public LEDPattern(int start, int end, Color[] totalPattern) {
    startPercentage = start;
    endPercentage = end;
    length = end - start + 1;
    this.totalPattern = totalPattern;
  }

  // "clear" update
  public void update() {
    setRange(0, length - 1, new Color(0, 0, 0));
  }

  public void set(int percentage, Color color) {
    totalPattern[startPercentage + percentage].set(color);
  }

  public void setRange(int start, int end, Color color) {
    for (int i = start; i <= end; i++) {
      set(i, color);
    }
  }
}
