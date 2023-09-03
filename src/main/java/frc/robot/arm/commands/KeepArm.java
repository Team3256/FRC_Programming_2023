// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import frc.robot.arm.Arm;
import frc.robot.helpers.ParentCommand;

public class KeepArm extends ParentCommand {
  private Arm armSubsystem;

  public KeepArm(Arm armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    addChildCommands(new KeepArmAtPosition(armSubsystem));
    super.initialize();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
