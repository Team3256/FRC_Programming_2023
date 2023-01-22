// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import java.util.List;

public class AutoChooser {
  private static SendableChooser<Command> autoChooser;

  public static SendableChooser<Command> getDefaultChooser() {

    autoChooser = new SendableChooser<>();
    return autoChooser;
  }

  public static void createIncrementalPaths(String commandName, List<Command> commands) {
    Command path = new InstantCommand();
    for (int i = 0; i < commands.size(); i++) {
      path = path.andThen(commands.get(i));
      autoChooser.addOption(commandName + "-" + i, path);
    }
  }

  public static void createIncrementalPaths(List<String> commandNames, List<Command> commands) {
    Command path = new InstantCommand();
    for (int i = 0; i < commands.size(); i++) {
      path = path.andThen(commands.get(i));
      autoChooser.addOption(commandNames.get(i), path);
    }
  }

  public static Command getCommand() {
    return autoChooser.getSelected();
  }
}
