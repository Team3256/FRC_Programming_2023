// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mole.Mole;

public class MoleOff extends CommandBase {
  private final Mole moleSubsystem;

  public MoleOff(Mole moleSubsystem) {
    this.moleSubsystem = moleSubsystem;
    addRequirements(moleSubsystem);
  }

  @Override
  public void initialize() {
    moleSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
