// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.patterns;

import frc.robot.drivers.Color;

/**
 * first half of pattern displays color of first ball in the robot second half of pattern displays
 * color of second ball in the robot (red or blue) pattern will take up 0 to 49 percentage range
 */
public class CubePattern extends LEDPattern {

  public CubePattern(Color[] totalPattern) {
    // percentage range that pattern is allocated to
    super(0, 49, totalPattern);
  }

  // event driven/state driven updates
  @Override
  public void update() {
    super.update();
    // set colouring of pattern
    setRange(0, 24, new Color(255, 0, 0));
    setRange(25, 49, new Color(0, 0, 255));
  }
}
