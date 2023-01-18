// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;

public class AutoChooser {
  private static SendableChooser<Command> autoChooser;

  public static SendableChooser<Command> getDefaultChooser() {

    autoChooser = new SendableChooser<>();
    return autoChooser;
  }

  public static Command getCommand() {
    return autoChooser.getSelected();
  }
}
