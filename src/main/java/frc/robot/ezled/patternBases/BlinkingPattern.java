// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.patternBases;

import frc.robot.ezled.patterns.OffPattern;

public class BlinkingPattern extends AnimatedPattern {
  private final int onFrames;

  /**
   * Blinking Pattern Animation
   *
   * @param onFrames number of frames that the main LED Pattern will be displayed
   * @param offFrames number of frames that nothing will be displayed
   */
  public BlinkingPattern(int onFrames, int offFrames) {
    super(onFrames + offFrames);
    this.onFrames = onFrames;
  }

  /**
   * Set the LED Pattern that will be displayed during the blink's on period
   *
   * @param ledPattern Main LED Pattern
   */
  protected void setMainLEDPattern(LEDPattern ledPattern) {
    createKeyFrame(0, ledPattern);
    createKeyFrame(onFrames, new OffPattern());
  }
}
