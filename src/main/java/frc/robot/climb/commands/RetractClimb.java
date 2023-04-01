// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climb.Climb;

public class RetractClimb extends CommandBase {
  private final Climb climbSubsystem;

  public RetractClimb(Climb climbSubsystem) {
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Climb Retracted Started");
    climbSubsystem.retractClimb();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Climb Retracted Ended");
    climbSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
