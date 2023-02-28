// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.climb.Climb;

public class DeployClimb extends CommandBase {
  private final Climb climbSubsystem;

  public DeployClimb(Climb climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    if (Constants.kDebugEnabled) {
      System.out.println(this.getName() + " started");
    }
    climbSubsystem.deployClimb();
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
