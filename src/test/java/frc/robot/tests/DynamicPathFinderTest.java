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
import frc.robot.auto.dynamicpathgeneration.DynamicPathFinder;

// TODO: make testIntersection() function that is called multiple times by testIntersections() to
// create cleaner code
public class DynamicPathFinderTest extends UnitTestBase {
  // @Test
  public void testIntersections() {
    Translation2d position1;
    Translation2d position2;
    boolean isIntersectingChargeStation;

    position1 = new Translation2d(2, 2.5);
    position2 = new Translation2d(6.2, 2.5);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(
        isIntersectingChargeStation, false, "Check normal intersection between charge station");

    position1 = new Translation2d(6.2, 4);
    position2 = new Translation2d(6.2, 2.5);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(
        isIntersectingChargeStation, true, "Check normal non-collision with charge station");

    position1 = new Translation2d(5.6, 2.3);
    position2 = new Translation2d(3.75, 1);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(
        isIntersectingChargeStation,
        true,
        "Check intersection with the bottom right corner of the charge station");

    position1 = new Translation2d(5.94, 1.96);
    position2 = new Translation2d(3.75, 1);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(
        isIntersectingChargeStation, false, "Check intersection with buffer distance works");

    position1 = new Translation2d(5.4, 0.7);
    position2 = new Translation2d(2.4, 0.7);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(isIntersectingChargeStation, false, "Check valid path is not flagged");

    // paranoia
    position1 = new Translation2d(2.08, 4.55);
    position2 = new Translation2d(5.12, 0.78);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(
        isIntersectingChargeStation,
        false,
        "Final check normal intersection between charge station");

    position1 = new Translation2d(7.53, 6.19);
    position2 = new Translation2d(6.10, 2.49);
    isIntersectingChargeStation =
        DynamicPathFinder.doesPathSegmentHitObstacles(position1, position2);
    assertEquals(
        isIntersectingChargeStation, true, "Final check normal non-collision with charge station");
  }
}
