// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mole.Mole;

public class ShootCube extends CommandBase {
  Mole moleSubsytem;
  double desiredSpeed;

  public ShootCube(Mole moleSubsystem, double desiredSpeed) {
    this.moleSubsytem = moleSubsystem;
    this.desiredSpeed = desiredSpeed;
    addRequirements(moleSubsystem);
  }

  public ShootCube(Mole moleSubsystem, Mole.MolePreset desiredMoleAngle) {
    this(moleSubsystem, desiredMoleAngle.desiredSpeed);
  }

  @Override
  public void initialize() {
    moleSubsytem.shootCube(desiredSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    moleSubsytem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
