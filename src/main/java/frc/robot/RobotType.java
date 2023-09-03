// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.swerve.SwerveConstants.kAlphaOffsets;
import static frc.robot.swerve.SwerveConstants.kFinalOffsets;
import static frc.robot.swerve.SwerveConstants.kZiptideOffsets;

import edu.wpi.first.math.geometry.Rotation2d;

public enum RobotType {
  ZIPTIDE {
    @Override
    public Rotation2d getOffset(int module) {
      return kZiptideOffsets[module];
    }
  },

  ALPHA {
    @Override
    public Rotation2d getOffset(int module) {
      return kAlphaOffsets[module];
    }
  },

  FINAL {
    @Override
    public Rotation2d getOffset(int module) {
      return kFinalOffsets[module];
    }
  };

  public abstract Rotation2d getOffset(int module);
}
