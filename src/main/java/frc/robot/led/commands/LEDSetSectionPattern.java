// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LEDStrip;
import frc.robot.led.patternBases.LEDPattern;

public class LEDSetSectionPattern extends CommandBase {
  private final LEDStrip LEDSubsystem;
  private final LEDPattern pattern;
  private int sectionID;

  public LEDSetSectionPattern(LEDStrip LEDSubsystem, int sectionID, LEDPattern pattern) {
    addRequirements(LEDSubsystem);
    this.LEDSubsystem = LEDSubsystem;
    this.sectionID = sectionID;
    this.pattern = pattern;
  }

  @Override
  public void initialize() {
    LEDSubsystem.set(sectionID, pattern);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
