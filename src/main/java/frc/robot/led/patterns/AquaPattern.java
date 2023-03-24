// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.led.patternBases.TimeFunctionPattern;

/** Blue Sin Wave Animation */
public class AquaPattern extends TimeFunctionPattern {
  private final double start = 114;
  private final double end = 160;

  public AquaPattern() {
    super(false);
  }

  private final double cycleTicks = 20;

  @Override
  protected double calculateA(int ticks) {
    return ((end - start) / 2 * Math.cos(ticks / cycleTicks) + start) / 360;
  }

  @Override
  protected double calculateB(int ticks) {
    return (0.15 / 2 * Math.cos(ticks / cycleTicks / 10) + 0.85);
  }

  @Override
  protected double calculateC(int ticks) {
    return 1;
  }
}
