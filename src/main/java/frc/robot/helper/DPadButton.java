// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helper;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * DPad Buttons are weird, called POV buttons they do not fully cooperate with our command-based
 * workflow.
 *
 * <p>This class allows us to use the buttons as normal buttons.
 */
public class DPadButton extends Trigger {
  CommandGenericHID joystick;
  Direction direction;

  public DPadButton(CommandGenericHID joystick, Direction direction) {
    this.joystick = joystick;
    this.direction = direction;
  }

  public enum Direction {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    final int direction;

    Direction(int direction) {
      this.direction = direction;
    }
  }

  public boolean get() {
    int dPadValue = joystick.getHID().getPOV();
    return (dPadValue == direction.direction)
        || (dPadValue == (direction.direction + 45) % 360)
        || (dPadValue == (direction.direction + 315) % 360);
  }
}
