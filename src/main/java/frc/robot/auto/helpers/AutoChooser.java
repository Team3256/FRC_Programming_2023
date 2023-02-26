// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.List;

public class AutoChooser {
  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static void createSingleDefaultPath(String commandName, Command command) {
    autoChooser.setDefaultOption(commandName, command);
  }

  public static void createSinglePath(String commandName, Command command) {
    autoChooser.addOption(commandName, command);
  }

  public static void addIncrementalPaths(
      Command intitialCommand, String commandName, List<Command> commands) {
    Command path = intitialCommand;
    for (int i = 0; i < commands.size(); i++) {
      path = path.andThen(commands.get(i));
      if (i == commands.size() - 1) {
        autoChooser.addOption(commandName + " - FULL", path);
      } else {
        autoChooser.addOption(commandName + " - " + i, path);
      }
    }
  }

  public static void addIncrementalPaths(
      Command intitialCommand, List<String> commandNames, List<Command> commands) {
    Command path = intitialCommand;
    for (int i = 0; i < commands.size(); i++) {
      path = path.andThen(commands.get(i));
      autoChooser.addOption(commandNames.get(i), path);
    }
  }

  public static void addIncrementalPaths(String commandName, List<Command> commands) {
    Command path = new InstantCommand();
    for (int i = 0; i < commands.size(); i++) {
      path = path.andThen(commands.get(i));
      if (i == commands.size() - 1) {
        autoChooser.addOption(commandName + " - FULL", path);
      } else {
        autoChooser.addOption(commandName + " - " + i, path);
      }
    }
  }

  public static void addIncrementalPaths(List<String> commandNames, List<Command> commands) {
    Command path = new InstantCommand();
    for (int i = 0; i < commands.size(); i++) {
      path = path.andThen(commands.get(i));
      autoChooser.addOption(commandNames.get(i), path);
    }
  }

  public static void sendChooserToDashboard(String name) {
    SmartDashboard.putData(name, autoChooser);
  }

  public static Command getCommand() {
    return autoChooser.getSelected();
  }
}
