// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.swerve.SwerveConstants.kXAutoBalanceVelocity;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.swerve.SwerveDrive;

public class AutoBalance extends DebugCommandBase {
  private final SwerveDrive swerveDrive;
  private TimedBoolean isBalanced;

  public AutoBalance(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.isBalanced = new TimedBoolean(swerveDrive::isNotTilted, 3);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    super.initialize();
    isBalanced.initialize();
  }

  @Override
  public void execute() {
    if (swerveDrive.isTiltedForward()) {
      swerveDrive.drive(new Translation2d(kXAutoBalanceVelocity, 0), 0, true, true);
    } else if (swerveDrive.isTiltedBackward()) {
      swerveDrive.drive(new Translation2d(kXAutoBalanceVelocity, 0), 0, true, true);
    } else {
      swerveDrive.drive(new Translation2d(0, 0), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    CommandScheduler.getInstance().schedule(new LockSwerveX(swerveDrive));
  }

  @Override
  public boolean isFinished() {
    isBalanced.update();
    return isBalanced.hasBeenTrueForThreshold();
  }
}
