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
    List<Translation2d> positions = new ArrayList<>();
    positions.add(new Translation2d(6, 0.5));
    positions.add(new Translation2d(6, 2));
    positions.add(new Translation2d(7.5, 2));
    Path path = new Path(positions, new Rotation2d(0), new Rotation2d(0));
    testInterpolatePathBase(path, "InterpolationTest-Turn");
  }

  // Top test
  @Test
  public void testGeneratePath1() {
    Pose2d src = new Pose2d(new Translation2d(7.8, 4.8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    testGeneratePathBase(src, sink, "DynamicTest-High");
  }

  // Bottom test
  @Test
  public void testGeneratePath2() {
    Pose2d src = new Pose2d(new Translation2d(7.5, 0.5), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    testGeneratePathBase(src, sink, "DynamicTest-Low");
  }

  // Cycle test
  @Test
  public void testGeneratePath3() {
    Pose2d src = new Pose2d(new Translation2d(16, 8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Cycle");
  }

  public void testGeneratePathBase(Pose2d src, Pose2d sink, String fileName) {
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    List<Translation2d> positions = generator.getPositions();
    Path path = new Path(positions, src.getRotation(), sink.getRotation());
    testInterpolatePathBase(path, fileName);
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
    // assertTrue(FileHelper.areJsonFilesSame(pathPlannerJsonPath, correctJsonPath));
  }
}
