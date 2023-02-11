// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.helpers.DynamicPathGenerator;
import org.junit.jupiter.api.Test;

import java.util.List;

public class DynamicPathGenerationTest {
	@Test
	public void testGeneratePath(){
		Pose2d src = new Pose2d(new Translation2d(6,3),new Rotation2d(0));
		Pose2d sink = new Pose2d(new Translation2d(2,3), new Rotation2d(0));
		DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
		List<PathPoint> path = generator.computePath();
	}
}
