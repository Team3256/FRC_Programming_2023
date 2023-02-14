// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.Rectangle2D;

public class Obstacle {
  private Rectangle2D.Double rectangle;

  public Obstacle(Translation2d topLeftCorner, double width, double height) {
    rectangle = new Rectangle2D.Double(topLeftCorner.getX(), topLeftCorner.getY(), width, height);
    System.out.println(rectangle);
  }

  public boolean containsPoint(Translation2d query) {
    return rectangle.contains(query.getX(), query.getY());
  }

  public boolean intersectsLineSegment(Translation2d start, Translation2d end) {
    return rectangle.intersectsLine(start.getX(), start.getY(), end.getX(), end.getY());
  }
}
