// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.swerve.SwerveConstants.AlphaOffsets;
import static frc.robot.swerve.SwerveConstants.ZipTideOffsets;

import edu.wpi.first.math.geometry.Rotation2d;

public enum RobotType {
  ZIPTIDE {
    @Override
    public Rotation2d getOffset(int module) {
      return ZipTideOffsets[module];
    }
  },

  ALPHA {
    @Override
    public Rotation2d getOffset(int module) {
      return AlphaOffsets[module];
    }
  };

  public abstract Rotation2d getOffset(int module);
}
