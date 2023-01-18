// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommandMarker {
  private Translation2d pos;
  private Translation2d endingPos;
  private Command command;

  public AutoCommandMarker(Translation2d pos, Command command) {
    this.pos = pos;
    this.command = command;
  }

  public AutoCommandMarker(Translation2d pos, Translation2d endingPos, Command command) {
    this.pos = pos;
    this.endingPos = endingPos;
    this.command = command;
  }

  public Command getCommand() {
    return command;
  }

  public Translation2d getPos() {
    return pos;
  }

  public Translation2d getEndingPos() {
    return endingPos;
  }
}
