// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

public class TrainingDataPoint {

  public double distance;
  public double stdDevXTranslation;
  public double stdDevYTranslation;
  public double stdDevAngle;

  public TrainingDataPoint(
      double distance, double stdDevXTranslation, double stdDevYTranslation, double stdDevAngle) {
    this.distance = distance;
    this.stdDevXTranslation = stdDevXTranslation;
    this.stdDevYTranslation = stdDevYTranslation;
    this.stdDevAngle = stdDevAngle;
  }
}
