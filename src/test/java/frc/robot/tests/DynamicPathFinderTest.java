// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.UnitTestBase;
import frc.robot.auto.helpers.DynamicPathFinder;

// TODO: make testIntersection() function that is called multiple times by testIntersections() to
// create cleaner code
public class DynamicPathFinderTest extends UnitTestBase {
  // @Test
  public void testIntersections() {
    Pose2d pose1;
    Pose2d pose2;
    boolean isIntersectingChargeStation;

    pose1 = new Pose2d(new Translation2d(2, 2.5), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(6.2, 2.5), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(
        isIntersectingChargeStation, false, "Check normal intersection between charge station");

    pose1 = new Pose2d(new Translation2d(6.2, 4), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(6.2, 2.5), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(
        isIntersectingChargeStation, true, "Check normal non-collision with charge station");

    pose1 = new Pose2d(new Translation2d(5.6, 2.3), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(3.75, 1), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(
        isIntersectingChargeStation,
        true,
        "Check intersection with the bottom right corner of the charge station");

    pose1 = new Pose2d(new Translation2d(5.94, 1.96), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(3.75, 1), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(
        isIntersectingChargeStation, false, "Check intersection with buffer distance works");

    pose1 = new Pose2d(new Translation2d(5.4, 0.7), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(2.4, 0.7), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(isIntersectingChargeStation, false, "Check valid path is not flagged");

    // paranoia
    pose1 = new Pose2d(new Translation2d(2.08, 4.55), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(5.12, 0.78), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(
        isIntersectingChargeStation,
        false,
        "Final check normal intersection between charge station");

    pose1 = new Pose2d(new Translation2d(7.53, 6.19), new Rotation2d());
    pose2 = new Pose2d(new Translation2d(6.10, 2.49), new Rotation2d());
    isIntersectingChargeStation = DynamicPathFinder.doesPathHitObstacles(pose1, pose2);
    assertEquals(
        isIntersectingChargeStation, true, "Final check normal non-collision with charge station");
  }
}
