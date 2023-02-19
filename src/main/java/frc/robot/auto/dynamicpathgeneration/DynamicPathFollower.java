// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueImportantLocations;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;

public class DynamicPathFollower {
  static void run(SwerveDrive swerveDrive) {
    // get src, sink
    Pose2d src = swerveDrive.getPose();
    int locationId = (int) SmartDashboard.getNumber("locationId", -1);
    if (locationId == -1) {
      System.out.println("locationId entered was invalid.");
      return;
    }
    Pose2d sink = kBlueImportantLocations[locationId];
    // get trajectory
    DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
    PathPlannerTrajectory trajectory = generator.getTrajectory();
    // create command that runs trajectory
    AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
    Command toRun = autoBuilder.createPathPlannerCommand(trajectory, false);
    // run command
    toRun.schedule();
  }
}
