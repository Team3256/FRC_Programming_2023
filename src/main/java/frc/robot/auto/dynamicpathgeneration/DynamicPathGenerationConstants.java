// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.Constants.FieldConstants.Community.*;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import java.util.ArrayList;

public final class DynamicPathGenerationConstants {
  public static final boolean kDynamicPathGenerationDebug = true;
  public static final double kCollisionBuffer = 0.1;
  public static final double kRobotRadius = 0.45 * Math.sqrt(2);
  public static final double kHitBoxResolution = 100;
  public static final double kControlPointScalar = 0.2;
  public static final Obstacle chargingStation = new Obstacle(kChargingStationCorners);
  public static final Obstacle[] obstacles = {chargingStation};
  public static final double searchLowX = 1.86;
  public static final double searchLowY = 0.51;
  public static final double searchResX = 0.1;
  public static final double searchResY = 0.1;
  public static final double searchHiX = 5.85;
  public static final double searchHiY = 4.85;
  public static final ArrayList<Translation2d> dynamicPathAllowedPositions = new ArrayList<>();
  public static final PathConstraints dynamicPathConstraints = new PathConstraints(5, 5);
  
}
