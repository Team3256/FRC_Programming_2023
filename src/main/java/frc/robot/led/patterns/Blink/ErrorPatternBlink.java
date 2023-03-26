// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns.Blink;

import static frc.robot.led.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.patternBases.BlinkingPattern;
import frc.robot.led.patternBases.LEDPattern;

/** Blinking Red */
public class ErrorPatternBlink extends BlinkingPattern {
  private class ErrorPattern extends LEDPattern {
    public ErrorPattern() {
      super();
      setPixelRange(1, kResolution, Color.kRed);
    }
  }

  public ErrorPatternBlink() {
    super(10, 5);
    setMainLEDPattern(new ErrorPattern());
  }
}
