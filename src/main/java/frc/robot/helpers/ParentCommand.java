// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ParentCommand extends DebugCommandBase {
  private Command childCommands = new InstantCommand();

  protected void addChildCommands(Command... commands) {
    childCommands = new ParallelCommandGroup(commands);
  }

  @Override
  public void initialize() {
    childCommands.schedule();
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    childCommands.cancel();
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return childCommands.isFinished();
  }
}
