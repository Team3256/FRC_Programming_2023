// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.intake.Intake;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IntakeUnitTests {
  public final double DELTA = 0.05;

  Intake intake;

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    new Intake();
  }

  @Test
  public void intakeTestForward() {
    IntakeForward command = new IntakeForward(intake);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command);

    double velocity = intake.getIntakeSpeed();
    Assertions.assertEquals(0.50, velocity, DELTA);
  }

  @Test
  public void intakeTestBackward() {
    Intake intake = new Intake();
    Outtake command = new Outtake(intake);
    CommandScheduler.getInstance().schedule(command);
    runScheduler2(3, command);

    double velocity = intake.getIntakeSpeed();
    Assertions.assertEquals(-0.50, velocity, DELTA);
  }

  private static void runScheduler(double seconds, IntakeForward command) {
    try {
      for (int i = 0; i < seconds * 1000 / 20; ++i) {
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        CommandScheduler.getInstance().run();
        command.initialize();
        Thread.sleep(20);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  private static void runScheduler2(double seconds, Outtake command) {
    try {
      for (int i = 0; i < seconds * 1000 / 20; ++i) {
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        CommandScheduler.getInstance().run();
        command.initialize();
        Thread.sleep(20);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
