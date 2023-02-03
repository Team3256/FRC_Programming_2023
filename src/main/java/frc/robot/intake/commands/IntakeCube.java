// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class IntakeCube extends CommandBase {

  private final Intake intake;

  public IntakeCube(Intake subsystem) {
    intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    intake.intakeCube();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
