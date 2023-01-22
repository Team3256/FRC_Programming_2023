// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patternBases;

public class AnimatedPattern extends LEDPattern {
  LEDPattern[] eventList;
  int ticks;
  int tick;

  public AnimatedPattern() {
    super();
    ticks = 1;
    eventList = new LEDPattern[ticks];
  }

  public AnimatedPattern(int ticks) {
    super();
    this.ticks = ticks;
    eventList = new LEDPattern[ticks];
  }

  @Override
  public void update() {
    pattern = eventList[tick].getPattern();
    tick = (tick + 1) % ticks;
  }

  public void setEvent(int tick, LEDPattern event) {
    eventList[tick] = event;
  }
}
