// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ThetaSupplier {
  Rotation2d rotationSupply(double now);

  void setTrajectoryDuration(double duration);
}