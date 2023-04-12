// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class TrajectoryMirrorer {
  public static PathPlannerState mirrorState(PathPlannerState state, Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return state;
    }

    PathPlannerState mirroredState = new PathPlannerState();
    Pose2d mirroredPose =
        new Pose2d(
            FieldConstants.kFieldLength - state.poseMeters.getX(),
            state.poseMeters.getY(),
            state.poseMeters.getRotation().unaryMinus());

    mirroredState.poseMeters = mirroredPose;
    mirroredState.holonomicRotation =
        state.holonomicRotation.plus(Rotation2d.fromDegrees(180)).unaryMinus();
    mirroredState.timeSeconds = state.timeSeconds;
    mirroredState.velocityMetersPerSecond = state.velocityMetersPerSecond;
    mirroredState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    mirroredState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;

    return mirroredState;
  }
}
