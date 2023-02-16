// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UnitTestBase;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ElevatorTests extends UnitTestBase {
  public final double DELTA = Units.inchesToMeters(2);

  private Elevator elevatorSubsystem;

  // Run simulateJava to test Elevator

  @BeforeAll
  public void setup() {
    super.setup();
    //    elevatorSubsystem = new Elevator();
  }

  @Test
  public void testElevatorHeightMax() {
    testElevatorHeight(0.997);
  }

  //  @Test
  //  public void testElevatorHeightMin() {
  //    testElevatorHeight(0.3048);
  //  }

  public void testElevatorHeight(double heightSetpointMeters) { // 1 meter
    Elevator elevatorSubsystem = new Elevator();
    Command command = new SetElevatorHeight(elevatorSubsystem, heightSetpointMeters);

    runScheduler(2, command, elevatorSubsystem);
    double height = elevatorSubsystem.getElevatorPosition();
    assertEquals(
        heightSetpointMeters,
        height,
        DELTA,
        "Setting elevator setpoint to " + heightSetpointMeters);
    //    elevatorSubsystem.zeroElevator();
  }
}
