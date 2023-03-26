// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import static frc.robot.led.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.patternBases.BlinkingPattern;
import frc.robot.led.patternBases.LEDPattern;

public class AutoMovePatternBlink extends BlinkingPattern {
  private class AutoMovePattern extends LEDPattern {
    public AutoMovePattern() {
      super();
      setPixelRange(1, kResolution, Color.kRoyalBlue);
    }
  }

  public AutoMovePatternBlink() {
    super(20, 20);
    setMainLEDPattern(new AutoMovePattern());
  }
}
