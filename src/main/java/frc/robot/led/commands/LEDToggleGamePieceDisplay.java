// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;

public class LEDToggleGamePieceDisplay extends CommandBase {
  private final LED ledStrip;

  public LEDToggleGamePieceDisplay(LED ledStrip) {
    addRequirements(ledStrip);
    this.ledStrip = ledStrip;
  }

  @Override
  public void initialize() {
    ledStrip.rainbow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
