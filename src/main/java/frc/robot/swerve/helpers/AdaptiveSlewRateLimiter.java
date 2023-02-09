// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import static frc.robot.elevator.ElevatorConstants.kRateLimiting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class AdaptiveSlewRateLimiter {
  private final double accelRateLimit;
  private final double decelRateLimit;
  private double prevVal;
  private double prevTime;

  public AdaptiveSlewRateLimiter(double accelRateLimit, double decelRateLimit) {
    this.accelRateLimit = Math.abs(accelRateLimit);
    this.decelRateLimit = Math.abs(decelRateLimit);

    prevVal = 0;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input
   * @return
   */
  public double calculate(double input, double elevatorPosition) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - this.prevTime;
    // constant kRateLimiting is to adjust the weight of the height of the elevator
    // TODO: Test to see if 1/x is the best response for rate limiting
    double currRateLimit =
        (Math.abs(input) > Math.abs(prevVal)
            ? kRateLimiting * accelRateLimit * (1 / elevatorPosition)
            : kRateLimiting * decelRateLimit * (1 / elevatorPosition));

    prevVal +=
        MathUtil.clamp(input - prevVal, -currRateLimit * elapsedTime, currRateLimit * elapsedTime);

    prevTime = currentTime;

    return prevVal;
  }

  public double calculate(double input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - this.prevTime;

    double currRateLimit = (Math.abs(input) > Math.abs(prevVal) ? accelRateLimit : decelRateLimit);

    prevVal +=
        MathUtil.clamp(input - prevVal, -currRateLimit * elapsedTime, currRateLimit * elapsedTime);

    prevTime = currentTime;

    return prevVal;
  }

  public void reset(double value) {
    prevVal = value;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }
}
