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
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.commands.SetElevatorExtension;
import frc.robot.elevator.commands.ZeroElevator;

public class ElevatorTests extends UnitTestBase {
  public final double DELTA = Units.inchesToMeters(2);

  private static Elevator elevatorSubsystem;

  // Run simulateJava to test Elevator
  // @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
    elevatorSubsystem = new Elevator();
  }

  // @Test
  public void testElevatorHeightMax() {
    testElevatorHeight(ElevatorConstants.kMaxExtension);
  }

  // @Test
  public void testElevatorZero() {
    Command zeroElevator = new ZeroElevator(elevatorSubsystem);
    System.out.println(elevatorSubsystem.getElevatorPosition());
    runScheduler(2, zeroElevator, elevatorSubsystem);

    double height = elevatorSubsystem.getElevatorPosition();
    System.out.println(height);
    assertEquals(ElevatorConstants.kMinExtension, height, DELTA, "Zeroing elevator");
  }

  public void testElevatorHeight(double heightSetpointMeters) { // 1 meter
    Command command = new SetElevatorExtension(elevatorSubsystem, heightSetpointMeters);

    runScheduler(2, command, elevatorSubsystem);
    double height = elevatorSubsystem.getElevatorPosition();
    assertEquals(
        heightSetpointMeters,
        height,
        DELTA,
        "Setting elevator setpoint to " + heightSetpointMeters);
  }
}
