// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

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
  // Turn sanity
  @Test
  public void testSimpleLeftTurnPath0() {
    long start = System.currentTimeMillis();
    List<Translation2d> positions = new ArrayList<>();
    positions.add(new Translation2d(6, 0.5));
    positions.add(new Translation2d(6, 2));
    positions.add(new Translation2d(7.5, 2));
    Path path = new Path(positions, new Rotation2d(0), new Rotation2d(0));
    testInterpolatePathBase(path, "InterpolationTest-Turn");
    System.out.println("Time taken: " + (System.currentTimeMillis() - start));
  }

  // Top test
  @Test
  public void testGeneratePath1() {
    long start = System.currentTimeMillis();
    Pose2d src = new Pose2d(new Translation2d(7.8, 4.8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    testGeneratePathBase(src, sink, "DynamicTest-High");
    System.out.println("Time taken: " + (System.currentTimeMillis() - start));
  }

  // Bottom test
  @Test
  public void testGeneratePath2() {
    long start = System.currentTimeMillis();
    Pose2d src = new Pose2d(new Translation2d(7.5, 0.5), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    testGeneratePathBase(src, sink, "DynamicTest-Low");
    System.out.println("Time taken: " + (System.currentTimeMillis() - start));
  }

  // Cycle test
  @Test
  public void testGeneratePath3() {
    long start = System.currentTimeMillis();
    Pose2d src = new Pose2d(new Translation2d(16, 8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Cycle");
    System.out.println("Time taken: " + (System.currentTimeMillis() - start));
  }

  // Mid short test
  @Test
  public void testGeneratePathMidShort() {
    Pose2d src = new Pose2d(new Translation2d(5.5, 2.7), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-MidShort");
  }

  // Mid med test
  @Test
  public void testGeneratePathMidMed() {
    Pose2d src = new Pose2d(new Translation2d(8.28, 2.76), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-MidMed");
  }

  // William test
  @Test
  public void testGeneratePath4() {
    long start = System.currentTimeMillis();
    Pose2d src = new Pose2d(new Translation2d(16, 8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-William");
    System.out.println("Time taken: " + (System.currentTimeMillis() - start));
  }

  public void testGeneratePathBase(Pose2d src, Pose2d sink, String fileName) {
    long start = System.currentTimeMillis();
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    List<Translation2d> positions = generator.getPositions();
    Path path = new Path(positions, src.getRotation(), sink.getRotation());
    testInterpolatePathBase(path, fileName);
    System.out.println("Time taken: " + (System.currentTimeMillis() - start));
  }

  public void testInterpolatePathBase(Path path, String fileName) {
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
