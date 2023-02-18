// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.UnitTestBase;
import frc.robot.auto.dynamicpathgeneration.DynamicPathConstants;
import org.junit.jupiter.api.Test;

public class DynamicPathObstacleTest extends UnitTestBase {
  @Test
  public void testChargeStationObstacle1() {
    Translation2d start = new Translation2d(7.50, 0.5);
    Translation2d end = new Translation2d(2.25, 4.5);
    assertEquals(
        true,
        DynamicPathConstants.kChargingStation.intersectsLineSegment(start, end),
        "Charge station interction test");
  }

  @Test
  public void testChargeStationObstacle2() {
    Translation2d start = new Translation2d(7.50, 0.5);
    Translation2d end = new Translation2d(2, 3);
    assertEquals(
        true,
        DynamicPathConstants.kChargingStation.intersectsLineSegment(start, end),
        "Charge station interction test");
  }

  @Test
  public void testChargeStationObstacle3() {
    Translation2d start = new Translation2d(2.20, 4.55);
    Translation2d end = new Translation2d(5.5, 4.55);
    assertEquals(
        false,
        DynamicPathConstants.kChargingStation.intersectsLineSegment(start, end),
        "Charge station interction test");
  }

  @Test
  public void testChargeStationObstacle4() {
    Translation2d start = new Translation2d(2.2, 4.2);
    Translation2d end = new Translation2d(2.2, 2);
    assertEquals(
        false,
        DynamicPathConstants.kChargingStation.intersectsLineSegment(start, end),
        "Charge station interction test");
  }

  @Test
  public void testChargeStationObstacle5() {
    Translation2d start = new Translation2d(2.81, 3.86);
    Translation2d end = new Translation2d(5.24, 3.86);
    assertEquals(
        true,
        DynamicPathConstants.kChargingStation.intersectsLineSegment(start, end),
        "Charge station interction test");
  }
}
