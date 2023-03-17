// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import frc.robot.led.patternBases.LEDPattern;

public class LEDSetAllSectionsPattern extends CommandBase {
  private final LED led;
  private final LEDPattern ledPattern;

  public LEDSetAllSectionsPattern(LED led, LEDPattern ledPattern) {
    addRequirements(led);
    this.led = led;
    this.ledPattern = ledPattern;
  }

  @Override
  public void initialize() {
    led.setAll(ledPattern);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
