// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.Constants.IntakeConstants.kIntakeForwardSpeed;
import static frc.robot.Constants.IntakeConstants.kOuttakeSpeed;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.intake.Intake;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IntakeUnitTests {
  public final double DELTA = 0.05;

  static Intake intakeSubsystem;

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    intakeSubsystem = new Intake();
  }

  @Test
  public void testForward() {
    IntakeForward command = new IntakeForward(intakeSubsystem);
    command.initialize();
    runScheduler(0.5);

    double velocity = intakeSubsystem.getIntakeSpeed();
    Assertions.assertEquals(kIntakeForwardSpeed, velocity, DELTA);
  }

  @Test
  public void testOuttake() {
    Outtake command = new Outtake(intakeSubsystem);
    command.initialize();
    runScheduler(0.5);

    double velocity = intakeSubsystem.getIntakeSpeed();
    Assertions.assertEquals(kOuttakeSpeed, velocity, DELTA);
  }

  private static void runScheduler(double seconds) {
    try {
      for (int i = 0; i < 10; ++i) {
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        CommandScheduler.getInstance().run();
        Thread.sleep((long) (seconds * 100));
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
