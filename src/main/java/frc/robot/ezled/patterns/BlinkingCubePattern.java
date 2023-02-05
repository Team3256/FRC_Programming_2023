// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.patterns;

import frc.robot.ezled.patternBases.BlinkingPattern;

/**
 * Blinking Purple
 */
public class BlinkingCubePattern extends BlinkingPattern {

  public BlinkingCubePattern() {
    super(35, 5);
    setMainLEDPattern(new CubePattern());
  }
}
