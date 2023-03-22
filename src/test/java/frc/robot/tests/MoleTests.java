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
import frc.robot.mole.commands.ShootCube;
import org.junit.jupiter.api.BeforeAll;

public class MoleTests extends UnitTestBase {
  private final double DELTA = 0.05;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
  }

  //  Tests are commented out because PID and feedforward have not been tuned yet

  //  @Test
  public synchronized void testPivotMid() {
    Mole mole = new Mole();
    ShootCube command = new ShootCube(mole, Mole.MolePreset.CUBE_MID);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command, mole);

    double position = mole.getMolePositionRads();
    assertEquals(kCubeMidAngle.getRadians(), position, DELTA, "testing Mole Pivot Mid");
  }

  //  @Test
  public synchronized void testPivotHigh() {
    Mole mole = new Mole();
    ShootCube command = new ShootCube(mole, Mole.MolePreset.CUBE_HIGH);
    CommandScheduler.getInstance().schedule(command);
    runScheduler(3, command, mole);

    double position = mole.getMolePositionRads();
    assertEquals(kCubeHighAngle.getRadians(), position, DELTA, "testing Mole Pivot High");
  }
}
