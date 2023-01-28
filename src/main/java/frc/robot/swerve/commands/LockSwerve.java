// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  public void initialize() {
    double inwardAngle = Math.atan(trackWidth / wheelBase);
    SwerveModuleState[] states = new SwerveModuleState[4];
    double[] offset = {0, 3 * Math.PI / 2, Math.PI / 2, Math.PI};

    for (int mod = 0; mod < 4; mod++) {
      states[mod] = new SwerveModuleState(1, new Rotation2d(inwardAngle + offset[mod]));
    }

    swerveDrive.setModuleStates(states);
  }

  @Override
  public void execute() {
    swerveDrive.setDriveMotorsNeutralMode(NeutralMode.Brake);
    swerveDrive.setAngleMotorsNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
