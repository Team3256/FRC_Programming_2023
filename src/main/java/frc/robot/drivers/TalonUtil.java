// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonUtil {
  /**
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(message + " " + errorCode, false);
    }
  }

  /**
   * checks the specified error code and throws an exception if there are any issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkErrorWithThrow(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      throw new RuntimeException(message + " " + errorCode);
    }
  }
}
