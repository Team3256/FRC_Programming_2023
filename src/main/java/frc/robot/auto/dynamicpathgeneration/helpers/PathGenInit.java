// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.*;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.dynamicPathAllowedPositions;

import edu.wpi.first.math.geometry.Translation2d;

public class PathGenInit {
  public static void init() {
    // init constants here since constants file doesn't allow it
    if (dynamicPathAllowedPositions.size() == 0) {
      for (double x = searchLowX; x < searchHiX; x += searchResX) {
        for (double y = searchLowY; y < searchHiY; y += searchResY) {
          boolean bad = false;
          for (int i = 0; i < kHitBoxResolution; i++) {
            double angle = i * 2 * Math.PI / kHitBoxResolution;
            double padding = kRobotRadius + kCollisionBuffer;
            if (chargingStation.containsPoint(
                new Translation2d(x + padding * Math.cos(angle), y + padding * Math.sin(angle))))
              bad = true;
          }
          if (!bad) dynamicPathAllowedPositions.add(new Translation2d(x, y));
        }
      }
    }
  }
}
