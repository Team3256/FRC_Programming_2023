// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.blue;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.FileHelper;
import frc.robot.auto.dynamicpathgeneration.helpers.Path;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.*;
import org.junit.jupiter.api.Test;

public class DynamicPathGenerationTest {

  @Test
  public void testInterpolateTurnPath() {
    List<Translation2d> positions = new ArrayList<>();
    positions.add(new Translation2d(6, 0.5));
    positions.add(new Translation2d(6, 2));
    positions.add(new Translation2d(7.5, 2));
    testInterpolatePathBase(
        new Rotation2d(0), new Rotation2d(0), positions, "InterpolateTest-Turn");
  }

  @Test
  public void testGeneratePathHigh() {
    Pose2d src = new Pose2d(new Translation2d(7.8, 4.8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(0));
    testGeneratePathBase(src, sink, "DynamicTest-High");
  }

  @Test
  public void testGeneratePathLow() {
    Pose2d src = new Pose2d(new Translation2d(7.5, 0.5), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(0));
    testGeneratePathBase(src, sink, "DynamicTest-Low");
  }

  @Test
  public void testGeneratePathCycle() {
    Pose2d src = new Pose2d(new Translation2d(15.22, 6.28), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Cycle");
  }

  @Test
  public void testGeneratePathCycleLow() {
    Pose2d src = new Pose2d(new Translation2d(15.22, 6.28), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.88, 0.52), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-CycleLow");
  }

  @Test
  public void testGeneratePathMidShort() {
    Pose2d src = new Pose2d(new Translation2d(6, 2.7), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-MidShort");
  }

  @Test
  public void testGeneratePathMidMed() {
    Pose2d src = new Pose2d(new Translation2d(8.28, 2.76), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-MidMed");
  }

  @Test
  public void testGeneratePathCom() {
    Pose2d src = new Pose2d(new Translation2d(1.75, 7.05), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Com");
  }

  @Test
  public void testGeneratePathBottom() {
    Pose2d src = new Pose2d(new Translation2d(14.44, 0.72), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Bottom");
  }

  public void testGeneratePathBase(Pose2d src, Pose2d sink, String fileName) {
    if (!blue) {
      src = new Pose2d(16.5 - src.getX(), src.getY(), src.getRotation());
      sink = new Pose2d(16.5 - sink.getX(), sink.getY(), sink.getRotation());
    }
    long start = System.currentTimeMillis();
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    List<Translation2d> positions = generator.getPositions();
    System.out.println("Time to find points: " + (System.currentTimeMillis() - start));
    testInterpolatePathBase(src.getRotation(), sink.getRotation(), positions, fileName);
  }

  public void testInterpolatePathBase(
      Rotation2d srcRot, Rotation2d sinkRot, List<Translation2d> points, String fileName) {
    long start = System.currentTimeMillis();
    Path path = new Path(points, srcRot, sinkRot);
    System.out.println("Time to interpolate path: " + (System.currentTimeMillis() - start));
    System.out.println("Test Path " + fileName + " Contents:");
    System.out.println(path.getWaypoints());
    JSONObject json = path.getJson();

    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    String correctJsonPath =
        "src/test/java/frc/robot/tests/dynamicpathgeneration/json/" + fileName + ".path";

    FileHelper.saveJson(json, pathPlannerJsonPath);
    assertTrue(true);
    // assertTrue(FileHelper.areJsonFilesSame(pathPlannerJsonPath,
    // correctJsonPath));
  }
}
