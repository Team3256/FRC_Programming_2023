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
  private Translation2d startingPosition;
  private Translation2d endingPosition;
  private double time;
  private Command command;

  public AutoCommandMarker(Translation2d startingPosition, double time, Command command) {
    this.startingPosition = startingPosition;
    this.command = command;
    this.time = time;
  }

  public AutoCommandMarker(
      Translation2d startingPosition, Translation2d endingPosition, double time, Command command) {
    this.startingPosition = startingPosition;
    this.endingPosition = endingPosition;
    this.command = command;
    this.time = time;
  }

  public Command getCommand() {
    return command;
  }

  public double getTime() {
    return time;
  }

  public Translation2d getStartingPosition() {
    return startingPosition;
  }

  public Translation2d getEndingPosition() {
    return endingPosition;
  }

  @Override
  public String toString() {
    return "Command Marker (Start: "
        + startingPosition
        + ", End: "
        + endingPosition
        + ", Command: "
        + command.getName()
        + ")";
  }
}
