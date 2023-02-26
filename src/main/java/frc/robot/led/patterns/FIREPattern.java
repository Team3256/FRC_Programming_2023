// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.led.patternBases.TimeFunctionPattern;

/** Blue Sin Wave Animation */
public class FIREPattern extends TimeFunctionPattern {
  public FIREPattern() {
    super(false);
  }

  private final double cycleTicks = 10;

  @Override
  protected double calculateA(int ticks) {
    return (7 / 2 * Math.cos(ticks / cycleTicks) + 7 / 2) / 360;
  }

  @Override
  protected double calculateB(int ticks) {
    return 1;
  }

  @Override
  protected double calculateC(int ticks) {
    return 1;
  }
}
