// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickTrigger extends Trigger {
  public JoystickTrigger() {}

  // TODO: Create a gradle task that when run will print the command name
  @Override
  public Trigger whileTrue(Command command) {
    super.whileTrue(command);
    System.out.println(command.getName());
    return this;
  }
}
