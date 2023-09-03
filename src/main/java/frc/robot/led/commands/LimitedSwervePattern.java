// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.commands;

import static frc.robot.led.LEDConstants.kCone;
import static frc.robot.led.LEDConstants.kCube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import java.util.function.BooleanSupplier;

public class LimitedSwervePattern extends CommandBase {
  private BooleanSupplier isCurrentPieceCone;

  public LimitedSwervePattern(LED ledSubsystem, BooleanSupplier isCurrentPieceCone) {
    this.isCurrentPieceCone = isCurrentPieceCone;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    if (isCurrentPieceCone.getAsBoolean()) {
      LED.LEDSegment.MainStrip.setStrobeAnimation(kCone, 0.7);
    } else {
      LED.LEDSegment.MainStrip.setStrobeAnimation(kCube, 0.7);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
