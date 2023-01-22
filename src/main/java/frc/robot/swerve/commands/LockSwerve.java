// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveDrive;

public class LockSwerve extends CommandBase {
  private final SwerveDrive swerveDrive;

  public LockSwerve(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double inwardAngle = Math.atan(trackWidth / wheelBase);
    SwerveModuleState[] states = new SwerveModuleState[4];
    double[] offset = {Math.PI, Math.PI / 2, 3 * Math.PI / 2, 0};

    for (int mod = 0; mod < 4; mod++) {
      states[mod] = new SwerveModuleState(lockedSpeed, new Rotation2d(inwardAngle + offset[mod]));
    }
    swerveDrive.setModuleStates(states);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
