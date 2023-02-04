// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static frc.robot.Constants.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorSetHeight;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ElevatorTests {
  public final double DELTA = 0.05;

  private static Elevator elevatorSubsystem;

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    elevatorSubsystem = new Elevator();
  }

  // TODO: Elevator command tests

  @Test
  public void testElevatorSetHeight() {
    ElevatorSetHeight command = new ElevatorSetHeight(elevatorSubsystem, 1);
    CommandScheduler.getInstance().schedule(command);
    //    command.execute();
    runScheduler(5, command);
    double height = elevatorSubsystem.getPosition();
    assertEquals(1, height, DELTA, "Setting elevator setpoint to 1");
  }

  private static void runScheduler(double seconds, Command command) {
    command.initialize();
    try {
      for (int i = 0; i < seconds * 1000 / 20; ++i) {
        HAL.simPeriodicBefore();
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        command.execute();
        if (command.isFinished()) command.end(false);
        Thread.sleep(20);
        HAL.simPeriodicAfter();
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
