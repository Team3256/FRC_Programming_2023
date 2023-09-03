// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import frc.robot.led.LEDConstants;

public class ColorFlowPattern extends CommandBase {

  public ColorFlowPattern(LED ledSubsystem) {
    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    LED.LEDSegment.MainStrip.setFlowAnimation(LEDConstants.kDefault, 0.8);
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
