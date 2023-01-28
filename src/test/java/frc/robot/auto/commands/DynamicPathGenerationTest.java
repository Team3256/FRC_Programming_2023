// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DynamicPathGenerationConstants;
import frc.robot.auto.helpers.AStar;
import java.util.function.BiFunction;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class DynamicPathGenerationTest {
  double[][] graph = DynamicPathGenerationConstants.adjacencyGraph;
  BiFunction<Translation2d, Translation2d, Double> heuristic =
      DynamicPathGenerationConstants.heuristic;

  @Test
  public void testAStar() {
    double distance = AStar.compute(graph, heuristic, 0, 8);
    System.out.println(distance);
    Assertions.assertEquals(0, distance);
  }
}
