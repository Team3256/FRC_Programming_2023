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
import edu.wpi.first.math.util.Units;
import frc.robot.auto.dynamicpathgeneration.helpers.Obstacle;
import java.util.ArrayList;

public final class DynamicPathGenerationConstants {
  public static final boolean kDynamicPathGenerationDebug = true;
  public static final double kCollisionBuffer = Units.inchesToMeters(10);
  public static final double kControlPointScalar = 0.2;
  public static final Obstacle chargingStation = new Obstacle(kChargingStationCorners);

  // Graph represnted below in (x, y)
  // <-> Represent edges
  // Graph is circular (first on top and bottom line connect, last on each line
  // connect)
  // 5.9, 4.3 <-> 4.5, 4.65 <-> 3.3, 4.65 <-> 2.2, 4.4 <-> 2.2, 2.7
  // |
  // 5.9, 1.3 <-> 4.5, 0.70 <-> 3.3, 0.70 <-> 2.2, 1.0 <-> 2.2, 2.7
  public static final double xi = 1.86;
  public static final double yi = 0.51;
  public static final double dx = 0.1;
  public static final double dy = 0.1;
  public static final double xf = 5.85;
  public static final double yf = 4.85;
  public static final ArrayList<Translation2d> dynamicPathAllowedPositions = new ArrayList<>();

  public static final PathConstraints dynamicPathConstraints = new PathConstraints(5, 5);
}
