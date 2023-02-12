// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.helpers.DynamicPathGenerator;
import frc.robot.helpers.FileHelper;
import frc.robot.helpers.Path;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.*;
import org.junit.jupiter.api.Test;

public class DynamicPathGenerationTest {
  @Test
  public void testSimpleLeftTurnPath0() {
    List<Pose2d> path = new ArrayList<>();
    path.add(new Pose2d(new Translation2d(6, 0.5), new Rotation2d(0)));
    path.add(new Pose2d(new Translation2d(6, 2), new Rotation2d(0)));
    path.add(new Pose2d(new Translation2d(7.5, 2), new Rotation2d(0)));
    testInterpolatePathBase(path, 0);
  }

  @Test
  public void testGeneratePath1() {
    Pose2d src = new Pose2d(new Translation2d(7.8, 4.8), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    testGeneratePathBase(src, sink, 1);
  }

  @Test
  public void testGeneratePath2() {
    Pose2d src = new Pose2d(new Translation2d(7.5, 0.5), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    testGeneratePathBase(src, sink, 2);
  }

  public void testGeneratePathBase(Pose2d src, Pose2d sink, int id) {
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    List<Pose2d> poseList = generator.computePath();
    System.out.println("Generated pose list:" + poseList);
    testInterpolatePathBase(poseList, id);
  }

  public void testInterpolatePathBase(List<Pose2d> poseList, int id) {
    Path path = new Path(poseList);
    JSONObject json = path.getJson();

    FileHelper.saveJson(
        json, "src/main/deploy/pathplanner/DynamicPathGenerationTest" + id + ".path");
    System.out.println("json:" + json);
    assert (true);
  }
}
