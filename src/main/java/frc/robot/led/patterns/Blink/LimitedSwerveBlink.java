// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns.Blink;

import frc.robot.led.patternBases.BlinkingPattern;
import frc.robot.led.patterns.ConePattern;
import frc.robot.led.patterns.CubePattern;
import java.util.function.BooleanSupplier;

public class LimitedSwerveBlink extends BlinkingPattern {
  public LimitedSwerveBlink(BooleanSupplier isCurrentPieceCone) {
    super(5, 5);
    if (isCurrentPieceCone.getAsBoolean()) {
      setMainLEDPattern(new ConePattern());
    } else {
      setMainLEDPattern(new CubePattern());
    }
  }
}
