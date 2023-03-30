// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UnitTestBase;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import org.junit.jupiter.api.BeforeAll;

public class ArmTests extends UnitTestBase {

  static final double DELTA = Units.degreesToRadians(3); // degrees

  private static Arm armSubsystem;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
    armSubsystem = new Arm();
  }

  // Arm Sim is setting to minimum angle. Tests don't really work
  // Test in simulateJava, gravity is weird in tests
  //  @Test
  public void testArmAngleVertical() {
    testArmAngle(Rotation2d.fromDegrees(90));
  }

  // @Test
  // public void testArmAngle12Deg() {
  // testArmAngle(Rotation2d.fromDegrees(12));
  // }

  // @Test
  // public void testArmAngleNeg5() {
  // testArmAngle(Rotation2d.fromDegrees(-5));
  // }

  public void testArmAngle(Rotation2d angle) {
    Command setAngleCommand = new SetArmAngle(armSubsystem, angle);
    runScheduler(3, setAngleCommand, armSubsystem);
    assertEquals(
        angle.getRadians(),
        armSubsystem.getArmPositionRads(),
        DELTA,
        "Set angle to " + angle.getRadians() + " radians");
    armSubsystem.off();
  }
}
