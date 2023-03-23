// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.helper;

public class TrainingDataPoint {
  public double distance;
  public double moleShooterRPM;

  public TrainingDataPoint(double distance, double moleShooterRPM) {
    this.distance = distance;
    this.moleShooterRPM = moleShooterRPM;
  }
}
