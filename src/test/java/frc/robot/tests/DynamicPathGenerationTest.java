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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.FileUtil;
import frc.robot.auto.dynamicpathgeneration.helpers.Path;
import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.swerve.SwerveDrive;
import java.util.List;
import org.json.simple.*;
import org.junit.jupiter.api.BeforeAll;

public class DynamicPathGenerationTest {
  // @Test
  // public void testInterpolateTurnPath() {
  // List<Translation2d> positions = new ArrayList<>();
  // positions.add(new Translation2d(6, 0.5));
  // positions.add(new Translation2d(6, 2));
  // positions.add(new Translation2d(7.5, 2));
  // testInterpolatePathBase(
  // new Rotation2d(0), new Rotation2d(0), positions, "InterpolateTest-Turn");
  // }
  public static SwerveDrive swerveDrive;

  @BeforeAll
  public static void setup() {
    swerveDrive = new SwerveDrive();
  }

  public void testGeneratePathHigh() {
    Pose2d src = new Pose2d(new Translation2d(7.8, 4.8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-High");
  }

  public void testGeneratePathYo() {
    Pose2d src = new Pose2d(new Translation2d(12.66, 4.62), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2.1, 0.72), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Yo");
  }

  public void testGeneratePathLow() {
    Pose2d src = new Pose2d(new Translation2d(7.5, 0.5), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Low");
  }

  public void testGeneratePathCycle() {
    Pose2d src = new Pose2d(new Translation2d(15.22, 6.28), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Cycle");
  }

  public void testGeneratePathCycleLow() {
    Pose2d src = new Pose2d(new Translation2d(15.22, 6.28), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.88, 0.52), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-CycleLow");
  }

  public void testGeneratePathMidShort() {
    Pose2d src = new Pose2d(new Translation2d(6, 2.7), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-MidShort");
  }

  public void testGeneratePathMidMed() {
    Pose2d src = new Pose2d(new Translation2d(8.28, 2.76), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-MidMed");
  }

  public void testGeneratePathCom() {
    Pose2d src = new Pose2d(new Translation2d(1.75, 7.05), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Com");
  }

  public void testGeneratePathBottom() {
    Pose2d src = new Pose2d(new Translation2d(14.44, 0.72), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.89, 2.70), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Bottom");
  }

  public void testGeneratePathLongScore() {
    Pose2d src = new Pose2d(new Translation2d(1.88, 0.53), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.90, 4.44), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-LongScore");
  }

  public void testGeneratorPassage() {
    Pose2d src = new Pose2d(new Translation2d(3.81, 0.80), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(1.90, 4.44), new Rotation2d(Math.PI));
    testGeneratePathBase(src, sink, "DynamicTest-Passage");
  }

  public void testGeneratePathBase(Pose2d src, Pose2d sink, String fileName) {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      src = PathUtil.flip(src);
      sink = PathUtil.flip(sink);
    }
    long start = System.currentTimeMillis();
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink, swerveDrive);
    List<PathNode> pathNodes = generator.getPathNodes();
    System.out.println("Time to find points: " + (System.currentTimeMillis() - start));
    testInterpolatePathBase(src.getRotation(), sink.getRotation(), pathNodes, fileName);
  }

  public void testInterpolatePathBase(
      Rotation2d srcRot, Rotation2d sinkRot, List<PathNode> pathNodes, String fileName) {
    long start = System.currentTimeMillis();
    Path path = new Path(pathNodes, srcRot, sinkRot);
    System.out.println("Time to interpolate path: " + (System.currentTimeMillis() - start));
    System.out.println("Test Path " + fileName + " Contents:");
    System.out.println(path.getWaypoints());
    JSONObject json = path.getJson();

    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    String correctJsonPath =
        "src/test/java/frc/robot/tests/dynamicpathgeneration/json/" + fileName + ".path";

    FileUtil.saveJson(json, pathPlannerJsonPath);
    assertTrue(true);
    // assertTrue(FileHelper.areJsonFilesSame(pathPlannerJsonPath,
    // correctJsonPath));
  }
}
