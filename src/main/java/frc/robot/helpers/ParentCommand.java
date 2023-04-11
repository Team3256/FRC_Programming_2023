// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.Arrays;

public class ParentCommand extends DebugCommandBase {
  private ArrayList<Command> childCommands = new ArrayList<>();

  protected void addChildCommands(Command... commands) {
    childCommands.addAll(Arrays.asList(commands));

    // add child command while parent command is running
    if (this.isScheduled()) {
      for (Command command : commands) {
        command.schedule();
      }
    }
  }

  @Override
  public void initialize() {
    for (Command childCommand : childCommands) {
      childCommand.schedule();
    }
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    for (Command childCommand : childCommands) {
      childCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    for (Command childCommand : childCommands) {
      if (!childCommand.isFinished()) return false;
    }
    return true;
  }
}
