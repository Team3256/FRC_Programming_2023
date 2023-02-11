package frc.robot.auto.helpers;

import org.junit.jupiter.api.Test;

import frc.robot.UnitTestBase;
import frc.robot.auto.helpers.DynamicPathFinder;

public class DynamicPathFinderTest extends UnitTestBase {
  @Test
  public static void testIntersections() {
    DynamicPathFinder.isPathConnectionValid(pose1, pose2);
  }

}
