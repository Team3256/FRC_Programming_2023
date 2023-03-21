// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static frc.robot.mole.MoleConstants.kMoleCubeSpeed;
import static frc.robot.mole.MoleConstants.kShootCubeAngle;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.UnitTestBase;
import frc.robot.mole.Mole;
import frc.robot.mole.commands.MoleIntakeCube;
import frc.robot.mole.commands.MoleOff;
import frc.robot.mole.commands.MoleShootCube;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class MoleTests extends UnitTestBase {
  private final double DELTA = 0.05;
  private static Mole mole;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
    mole = new Mole();
  }

  @Test
  public void testMoleIntakeCube() {
    MoleIntakeCube command = new MoleIntakeCube(mole);
    command.initialize();
    runScheduler(1, command, mole);

    double velocity = mole.getMoleSpeed();
    assertEquals(kMoleCubeSpeed, velocity, DELTA, "testing Mole Intake Cube");
  }

  @Test
  public void testMoleOff() {
    MoleOff command = new MoleOff(mole);
    command.initialize();
    runScheduler(1, command, mole);

    double velocity = mole.getMoleSpeed();
    assertEquals(0, velocity, 0, "Turning off Mole");
  }

  @Test
  public void testMoleShootCube() {
    MoleShootCube command = new MoleShootCube(mole, kShootCubeAngle);
    command.initialize();
    runScheduler(1, command, mole);

    double position = Math.toDegrees(mole.getMolePivotPositionRadians());
    double velocity = mole.getMoleSpeed();
    assertEquals(-kMoleCubeSpeed, velocity, DELTA, "Testing Mole shooting speed");
    assertEquals(kShootCubeAngle.getDegrees(), position, DELTA, "Testing Mole angle");
  }
}
