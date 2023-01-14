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
  private Translation2d marker;
  private Translation2d endingMarker;
  private Command command;

  public AutoCommandMarker(Translation2d marker, Command command) {
    this.marker = marker;
    this.command = command;
  }

  public AutoCommandMarker(Translation2d marker, Translation2d endingMarker, Command command) {
    this.marker = marker;
    this.endingMarker = endingMarker;
    this.command = command;
  }

  public Command getCommand() {
    return command;
  }

  public Translation2d getMarker() {
    return marker;
  }

  public Translation2d getEndingMarker() {
    return endingMarker;
  }
}
