// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.swerve.SwerveConstants.kXAutoBalanceVelocityMeters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.swerve.SwerveDrive;

public class AutoBalance extends DebugCommandBase {
  private final SwerveDrive swerveDrive;
  private Timer balancedTimer;

  public AutoBalance(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.balancedTimer = new Timer();

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    super.initialize();
    balancedTimer.start();
  }

  @Override
  public void execute() {
    if (swerveDrive.isTiltedForward()) {
      balancedTimer.reset();
      swerveDrive.drive(new Translation2d(-kXAutoBalanceVelocityMeters, 0), 0, true, true);
    } else if (swerveDrive.isTiltedBackward()) {
      balancedTimer.reset();
      swerveDrive.drive(new Translation2d(kXAutoBalanceVelocityMeters, 0), 0, true, true);
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
    return swerveDrive.isNotTilted() && balancedTimer.get() >= 2;
  }
}
