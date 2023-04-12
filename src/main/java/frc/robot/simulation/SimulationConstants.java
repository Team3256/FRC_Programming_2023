// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.util.Units;

public class SimulationConstants {
  public static final double kRobotSimWindowWidth = 2.5;
  public static final double kRobotSimWindowHeight = 2.5;
  public static final double kGoalStationX = 0.9 * kRobotSimWindowWidth;

  public static final double kRootX = 0.5;
  public static final double kRootY = 0;
  public static final double kElevatorRootX = kRootX + Units.inchesToMeters(4.159);
  public static final double kElevatorRootY = Units.inchesToMeters(2.773);

  public static final double kConeTipHeight = 0.33; // official height
  public static final double kConeTipLineWidth = 20;
  public static final double kConeBaseHeight = 0.08;
  public static final double kConeBaseLineWidth = 50;

  public static final double kCubeBase = 0.24; // official height
  public static final double kCubeLineWidth = 60;

  public static final double kElevatorLineWidth = 10;
  public static final double kArmLineWidth = 10;
  public static final double kIntakeLineWidth = 3;

  public static final double kSimulateDelta = 0.020;
  public static final int kVoltage = 12;
  public static final double kIntakeRadius = 0.1;

  public static final double kArmStartPosition = 8.582;
  public static final double kArmPivotHeight = 4.25;
  public static final double kMinElevatorExtension = Units.inchesToMeters(29);
}
