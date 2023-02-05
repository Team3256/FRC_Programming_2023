// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.patterns;

import frc.robot.ezled.patternBases.TimeFunctionPattern;

/** Blue Sin Wave Animation */
public class ColorChaseBluePattern extends TimeFunctionPattern {
  public ColorChaseBluePattern() {
    super(true);
  }

  private final double cycleTicks = 10;

  @Override
  protected double calculateA(int ticks) {
    return 0.25 + 0.25 * Math.cos(ticks / cycleTicks);
  }

  @Override
  protected double calculateB(int ticks) {
    return 0.25 + 0.25 * Math.cos(ticks / cycleTicks);
  }

  @Override
  protected double calculateC(int ticks) {
    return 1;
  }
}
