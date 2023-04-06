// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.led.LED;
import frc.robot.led.patternBases.LEDPattern;

public class LEDSetAllSectionsPattern extends DebugCommandBase {
  private final LED led;
  private final LEDPattern ledPattern;

  public LEDSetAllSectionsPattern(LED led, LEDPattern ledPattern) {
    this.led = led;
    this.ledPattern = ledPattern;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    super.initialize();
    led.setAll(ledPattern);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
