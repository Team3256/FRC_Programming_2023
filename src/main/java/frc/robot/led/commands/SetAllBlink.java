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

public class SetAllBlink extends CommandBase {
  private Color color;
  private double speed = 0.5;

  public SetAllBlink(LED ledSubsystem, Color color) {
    this.color = color;
    addRequirements(ledSubsystem);
  }

  public SetAllBlink(LED ledSubsystem, Color color, double speed) {
    this(ledSubsystem, color);
    this.speed = speed;
  }

  @Override
  public void initialize() {
    LED.LEDSegment.MainStrip.setStrobeAnimation(color, speed);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
