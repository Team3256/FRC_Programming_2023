// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.simplepathgeneration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;

public class SimpleGoToRelative {
  static Command run(SwerveDrive swerveDrive, Translation2d translation2d) {
    return SimpleGoToAbsolute.run(
        swerveDrive.getPose(),
        swerveDrive.getPose().plus(new Transform2d(translation2d, new Rotation2d(0))),
        swerveDrive);
  }
}
