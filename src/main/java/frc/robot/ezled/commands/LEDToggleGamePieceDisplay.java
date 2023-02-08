// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ezled.EZLED;

public class LEDToggleGamePieceDisplay extends CommandBase {
  private final EZLED ledStrip;

  public LEDToggleGamePieceDisplay(EZLED ledStrip) {
    addRequirements(ledStrip);
    this.ledStrip = ledStrip;
  }

  @Override
  public void initialize() {
    ledStrip.toggleGamePiece();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
