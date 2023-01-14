// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import frc.robot.swerve.SwerveDrive;

public class Paths {
  private static TrajectoryFactory trajectoryFactory;

  public static void initialize(SwerveDrive drive) {
    trajectoryFactory =
        trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
  }
}
