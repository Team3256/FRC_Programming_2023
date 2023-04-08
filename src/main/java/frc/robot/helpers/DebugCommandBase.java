// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public abstract class DebugCommandBase extends CommandBase {
  @Override
  public void initialize() {
    if (Constants.kDebugEnabled) {
      System.out.println(
          "Command Started: " + this.getName() + ", Timestamp: " + Timer.getFPGATimestamp());
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.kDebugEnabled) {
      System.out.println(
          "Command Ended: "
              + this.getName()
              + ", Interrupted?: "
              + interrupted
              + ", Time: "
              + Timer.getFPGATimestamp());
    }
  }
}
