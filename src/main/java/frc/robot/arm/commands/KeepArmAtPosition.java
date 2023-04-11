// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.arm.Arm;
import frc.robot.helpers.DebugCommandBase;

public class KeepArmAtPosition extends DebugCommandBase {
  private Arm armSubsystem;

  public KeepArmAtPosition(Arm armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    new SetArmAngle(armSubsystem, new Rotation2d(armSubsystem.getArmPositionElevatorRelative()))
        .repeatedly()
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
