// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class UnitTestBase {
  public static void initilizeSetup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
  }

  static void runScheduler(double seconds, Command command, Subsystem subsystem) {
    command.initialize();
    try {
      for (int i = 0; i < seconds * 1000 / 20; ++i) {
        HAL.simPeriodicBefore();
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        command.execute();
        subsystem.simulationPeriodic();
        if (command.isFinished()) command.end(false);
        Thread.sleep(20);
        HAL.simPeriodicAfter();
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
