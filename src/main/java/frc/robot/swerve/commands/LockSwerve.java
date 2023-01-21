// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  public void initialize() {
    double inwardAngle = Math.tan(trackWidth / wheelBase);

    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int mod = 0; mod < 4; mod++) {
      states[mod] = new SwerveModuleState(lockedSpeed, new Rotation2d(inwardAngle));
      inwardAngle += Math.PI / 2;
      swerveDrive.setModuleStates(states);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
