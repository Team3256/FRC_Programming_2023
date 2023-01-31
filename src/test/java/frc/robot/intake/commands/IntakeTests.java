// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.Constants.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.intake.Intake;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IntakeTests {
  public final double DELTA = 0.05;

  private static Intake intakeSubsystem;

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    intakeSubsystem = new Intake();
  }

  @Test
  public void testIntakeCube() {
    IntakeCube command = new IntakeCube(intakeSubsystem);
    command.initialize();
    runScheduler(1);

    double velocity = intakeSubsystem.getIntakeSpeed();
    assertEquals(kIntakeCubeSpeed, velocity, DELTA);
  }

  @Test
  public void testIntakeCone() {
    IntakeCone command = new IntakeCone(intakeSubsystem);
    command.initialize();
    runScheduler(1);

    double velocity = intakeSubsystem.getIntakeSpeed();
    assertEquals(kIntakeConeSpeed, velocity, DELTA);
  }

  private static void runScheduler(double seconds) {
    try {
      for (int i = 0; i < seconds * 1000 / 20; ++i) {
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        CommandScheduler.getInstance().run();
        Thread.sleep(20);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
