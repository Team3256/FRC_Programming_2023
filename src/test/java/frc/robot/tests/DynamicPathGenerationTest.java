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
import java.util.List;
import org.json.simple.*;
import org.junit.jupiter.api.Test;

public class DynamicPathGenerationTest {
  @Test
  public void testGeneratePath() {
    Pose2d src = new Pose2d(new Translation2d(8.46, 0.78), new Rotation2d(0));
    Pose2d sink = new Pose2d(new Translation2d(2, 3), new Rotation2d(0));
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    List<Pose2d> path = generator.computePath();
    System.out.println("TEST Final path:" + path);
    Path pathPath = new Path(path);
    System.out.println(pathPath);
    JSONObject json = pathPath.getJson();

    FileHelper.saveJson(json, "src/main/deploy/pathplanner/DynamicPathGenerationTest.path");
    System.out.println("json:" + json);
    assert (true);
  }
}
