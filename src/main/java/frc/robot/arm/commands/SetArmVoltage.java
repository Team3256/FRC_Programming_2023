// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import frc.robot.arm.Arm;
import frc.robot.helpers.DebugCommandBase;

public class SetArmVoltage extends DebugCommandBase {
  private final Arm armSubsystem;
  private final double volts;

  public SetArmVoltage(Arm armSubsystem, double volts) {
    this.armSubsystem = armSubsystem;
    this.volts = volts;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    armSubsystem.setInputVoltage(volts);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    armSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
