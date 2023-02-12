// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TransHelper {
  public static Rotation2d angleBetweenVectors(Translation2d u, Translation2d v) {
    double numerator = u.getX() * v.getX() + u.getY() * v.getY();
    double denominator = u.getNorm() * v.getNorm();

    return Rotation2d.fromRadians(Math.acos(numerator / denominator));
  }

  public static Translation2d projectUonV(Translation2d u, Translation2d v) {
    double vMagnitude = v.getNorm();
    Translation2d vUnitVector = v.div(vMagnitude);
    double uDotVOverMagnitudeV = (u.getX() * v.getX() + u.getY() * v.getY()) / vMagnitude;
    return vUnitVector.times(uDotVOverMagnitudeV);
  }
}
