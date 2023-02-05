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
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  }

  // TODO: Elevator command tests

  @Test
  public void testElevatorSetHeight1() { // 1 meter
    elevatorSubsystem = new Elevator();
    ElevatorSetHeight command = new ElevatorSetHeight(elevatorSubsystem, 1);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command, elevatorSubsystem);
    double height = elevatorSubsystem.getElevatorPosition();
    System.out.println(height);
    assertEquals(1, height, DELTA, "Setting elevator setpoint to 1");
  }

  //  @Test
  //  public void testElevatorSetHeight2() { // 0.5 meters
  //    elevatorSubsystem = new Elevator();
  //    ElevatorSetHeight command = new ElevatorSetHeight(elevatorSubsystem, 0.5);
  //    CommandScheduler.getInstance().schedule(command);
  //    runScheduler(3, command, elevatorSubsystem);
  //    double height = elevatorSubsystem.getElevatorPosition();
  //    System.out.println(height);
  //    assertEquals(0.5, height, DELTA, "Setting elevator setpoint to 0.5");
  //  }
  //
  //  @Test
  //  public void testElevatorSetHeight3() { // 0.25 meters
  //    elevatorSubsystem = new Elevator();
  //    ElevatorSetHeight command = new ElevatorSetHeight(elevatorSubsystem, 0.25);
  //    CommandScheduler.getInstance().schedule(command);
  //    runScheduler(3, command, elevatorSubsystem);
  //    double height = elevatorSubsystem.getElevatorPosition();
  //    System.out.println(height);
  //    assertEquals(0.25, height, DELTA, "Setting elevator setpoint to 0.25");
  //  }
  //
  //  @Test
  //  public void testElevatorSetHeight4() { // 0.75 meters
  //    elevatorSubsystem = new Elevator();
  //    ElevatorSetHeight command = new ElevatorSetHeight(elevatorSubsystem, 0.75);
  //    CommandScheduler.getInstance().schedule(command);
  //    runScheduler(3, command, elevatorSubsystem);
  //    double height = elevatorSubsystem.getElevatorPosition();
  //    System.out.println(height);
  //    assertEquals(0.75, height, DELTA, "Setting elevator setpoint to 0.75");
  //  }

  private static void runScheduler(double seconds, Command command, Subsystem subsystem) {
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
