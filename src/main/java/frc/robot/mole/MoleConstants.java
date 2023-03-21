// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.drivers.CanDeviceId;

public final class MoleConstants {

  public static final int kMoleMotorID = 31;
  public static final int kMolePivotMotorID = 32;

  public static final String kMoleCANBus = "mani";
  public static final String kMolePivotCANBus = "mani";

  public static final CanDeviceId kMoleCANDevice = new CanDeviceId(kMoleMotorID, kMoleCANBus);
  public static final CanDeviceId kMolePivotCANDevice =
      new CanDeviceId(kMolePivotMotorID, kMolePivotCANBus);

  public static final double kMoleCubeSpeed = 0.9;
  public static final double kMolePivotSpeed = 0.9;

  public static final double kMoleKeepingCurrent = 2;
  public static final double kMolePivotKeepingCurrent = 2;

  public static final double kMoleCurrentSpikingThreshold = 40;
  public static final double kMolePivotCurrentSpikingThreshold = 40;

  public static final double kMolePivotGearing = 60;

  public static Rotation2d kDefaultMoleAngle = Rotation2d.fromDegrees(0);
  public static Rotation2d kShootCubeAngle = Rotation2d.fromDegrees(25); // change with testing
  public static Rotation2d kCubeMidRotation = Rotation2d.fromDegrees(45);
  public static Rotation2d kCubeHighRotation = Rotation2d.fromDegrees(70);

  public static Rotation2d kMolePivotAngleTolerance = Rotation2d.fromDegrees(5);
}
