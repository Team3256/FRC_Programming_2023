// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.swerve.SwerveConstants.kXAutoBalanceVelocity;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.SwerveDrive;

public class AutoBalance extends CommandBase {
  private final SwerveDrive swerveDrive;

  public AutoBalance(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (swerveDrive.isTiltedForward()) {
      swerveDrive.drive(new Translation2d(-kXAutoBalanceVelocity, 0), 0, true, true);
    } else {
      swerveDrive.drive(new Translation2d(kXAutoBalanceVelocity, 0), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new LockSwerveX(swerveDrive));
  }

  @Override
  public boolean isFinished() {
    return swerveDrive.isNotTilted();
  }
}
