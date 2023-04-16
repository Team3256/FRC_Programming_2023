// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import java.awt.Color;

public class BandPattern extends CommandBase {
  private Color color;

  public BandPattern(LED ledSubsystem, Color color) {
    this.color = color;
    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    LED.LEDSegment.MainStrip.setBandAnimation(color, 0.5);
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
