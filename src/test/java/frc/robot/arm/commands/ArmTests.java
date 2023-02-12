// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.UnitTestBase;
import frc.robot.arm.Arm;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ArmTests extends UnitTestBase {

  private static final double DELTA = 0.05;
  static Arm armSubsystem;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
  }

  // SetArmAngle test (one routine)

  @Test
  public void armAngleRoutine() {

    Rotation2d firstSet = new Rotation2d(600.0);
    Rotation2d secondSet = new Rotation2d(0.0);
    Rotation2d thirdSet = new Rotation2d(1200);
    SetArmAngle SetArmAngleToFirstSet = new SetArmAngle(armSubsystem, firstSet);
    SetArmAngleToFirstSet.schedule();
    runScheduler(1.5, SetArmAngleToFirstSet, armSubsystem);
    assertEquals(
        armSubsystem.getArmPosition(),
        firstSet.getDegrees(),
        DELTA,
        "Set angular velocity to " + firstSet.getDegrees() + " RPM");

    SetArmAngle SetArmAngleToSecondSet = new SetArmAngle(armSubsystem, secondSet);
    SetArmAngleToSecondSet.schedule();
    runScheduler(1, SetArmAngleToSecondSet, armSubsystem);
    assertEquals(
        armSubsystem.getArmPosition(),
        secondSet.getDegrees(),
        DELTA,
        "Set angular velocity to " + secondSet.getDegrees() + " RPM");

    SetArmAngle SetArmAngleToThirdSet = new SetArmAngle(armSubsystem, thirdSet);
    SetArmAngleToThirdSet.schedule();
    runScheduler(1.5, SetArmAngleToThirdSet, armSubsystem);
    assertEquals(
        armSubsystem.getArmPosition(),
        thirdSet.getDegrees(),
        DELTA,
        "Set angular velocity to " + thirdSet.getDegrees() + " RPM");
    armSubsystem.off();
  }
}
