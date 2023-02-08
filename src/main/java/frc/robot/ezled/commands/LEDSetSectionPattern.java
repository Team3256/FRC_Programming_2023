// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ezled.EZLED;
import frc.robot.ezled.patternBases.LEDPattern;

public class LEDSetSectionPattern extends CommandBase {
  private final EZLED ledStrip;
  private final LEDPattern ledPattern;
  private final int sectionId;

  public LEDSetSectionPattern(EZLED ledStrip, int sectionID, LEDPattern ledPattern) {
    addRequirements(ledStrip);
    this.ledStrip = ledStrip;
    this.sectionId = sectionID;
    this.ledPattern = ledPattern;
  }

  @Override
  public void initialize() {
    ledStrip.set(sectionId, ledPattern);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
