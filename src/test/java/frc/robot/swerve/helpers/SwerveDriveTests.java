// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.UnitTestBase;
import frc.robot.swerve.SwerveConstants;
import org.junit.jupiter.api.Test;

public class SwerveDriveTests extends UnitTestBase {
  @Test
  void TestModuleA() {
    SwerveModule frontLeftModule = new SwerveModule(0, SwerveConstants.FrontLeft.constants);
    double speed = 100;
    Rotation2d rotation = new Rotation2d(45);
    SwerveModuleState state = new SwerveModuleState(speed, rotation);
    frontLeftModule.setDesiredState(state, true);
//    frontLeftModule.feedforward./
//    frontLeftModule.feedforward();

  }
}
