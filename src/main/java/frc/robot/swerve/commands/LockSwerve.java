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
  private final SwerveDrive swerveSubsystem;

  public LockSwerve(SwerveDrive swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    double inwardAngle = Math.atan(trackWidth / wheelBase);
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int mod = 0; mod < 4; mod++) {
      states[mod] = new SwerveModuleState(1, new Rotation2d(inwardAngle + lockAngleOffsets[mod]));
    }

    swerveSubsystem.setDesiredAngleState(states);
    swerveSubsystem.setDriveMotorsNeutralMode(NeutralMode.Brake);
    swerveSubsystem.setAngleMotorsNeutralMode(NeutralMode.Brake);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
