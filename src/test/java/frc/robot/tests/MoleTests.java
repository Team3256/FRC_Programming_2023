// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static frc.robot.mole.MoleConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.UnitTestBase;
import frc.robot.mole.Mole;
import frc.robot.mole.commands.MoleIntakeCube;
import frc.robot.mole.commands.MoleShootCube;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class MoleTests extends UnitTestBase {
  private final double DELTA = 0.05;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
  }

  @Test
  public synchronized void testMoleIntakeCube() {
    Mole mole = new Mole();
    MoleIntakeCube command = new MoleIntakeCube(mole);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command, mole);

    double speed = mole.getMoleSpeed();
    assertEquals(kMoleCubeSpeed, speed, DELTA, "testing Mole Intake Cube");
  }

  //  @Test
  public synchronized void testMolePivotMid() {
    Mole mole = new Mole();
    MoleShootCube command = new MoleShootCube(mole, kCubeMidRotation);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command, mole);

    double position = mole.getMolePivotPositionRadians();
    assertEquals(kCubeMidRotation.getRadians(), position, DELTA, "testing Mole Pivot Mid");
  }

  //  @Test
  public synchronized void testMolePivotHigh() {
    Mole mole = new Mole();
    MoleShootCube command = new MoleShootCube(mole, kCubeHighRotation);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command, mole);

    double position = mole.getMolePivotPositionRadians();
    assertEquals(kCubeHighRotation.getRadians(), position, DELTA, "testing Mole Pivot Mid");
  }
}
