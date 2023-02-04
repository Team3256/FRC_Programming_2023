// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

import frc.robot.drivers.Color;

import static frc.robot.Constants.LEDConstants.kResolution;

public class AnimatedPattern extends LEDPattern {
  private Color[][] eventList;
  private final int ticks;
  private int tick = 0;

  public AnimatedPattern(int ticks) {
    super();
    this.ticks = ticks;
    eventList = new Color[ticks][kResolution];
  }

  @Override
  public void update() {
    if (eventList[tick] != null) setPattern(eventList[tick]);
    tick = (tick + 1) % ticks;
  }

  protected void setEvent(int tick, Color[] event) {
    eventList[tick] = event;
  }
}
