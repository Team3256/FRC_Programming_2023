// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.climb.commands;

import frc.robot.climb.Climb;
import frc.robot.helpers.DebugCommandBase;

public class DeployClimb extends DebugCommandBase {
  private final Climb climbSubsystem;

  public DeployClimb(Climb climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    climbSubsystem.deployClimb();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    climbSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
