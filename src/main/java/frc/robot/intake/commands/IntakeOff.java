// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.intake.Intake;

public class IntakeOff extends DebugCommandBase {
  private final Intake intakeSubsystem;

  public IntakeOff(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
