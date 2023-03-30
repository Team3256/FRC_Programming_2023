// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.awt.geom.Rectangle2D;

public class Obstacle {
  private final String name;
  private final Rectangle2D.Double rectangle;

  // Rectangle2D.Double actually uses bottom left contrary to their docs
  public Obstacle(Translation2d bottomLeftCorner, double width, double height, String name) {
    this.name = name;
    rectangle =
        new Rectangle2D.Double(bottomLeftCorner.getX(), bottomLeftCorner.getY(), width, height);
  }

  public boolean containsPoint(Translation2d query) {
    return rectangle.contains(query.getX(), query.getY());
  }

  public boolean intersectsLineSegment(Translation2d start, Translation2d end) {
    return rectangle.intersectsLine(start.getX(), start.getY(), end.getX(), end.getY());
  }

  /** return red version of blue obstacles */
  public Obstacle getRedVersion() {
    double newBottomLeftCornerX =
        Constants.FieldConstants.kFieldLength - rectangle.getX() - rectangle.width;
    return new Obstacle(
        new Translation2d(newBottomLeftCornerX, rectangle.getY()),
        rectangle.width,
        rectangle.height,
        "RED" + name);
  }

  public String toString() {
    return name;
  }
}
