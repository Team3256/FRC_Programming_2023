// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {
  // return CCW rotation between u and v in degrees
  public static Rotation2d angleBetweenVectorsCCW(Translation2d u, Translation2d v) {
    double dot = u.getX() * v.getX() + u.getY() * v.getY();
    double det = u.getX() * v.getY() - u.getY() * v.getX();

    return Rotation2d.fromRadians((Math.atan2(det, dot) + Math.PI * 2) % (Math.PI * 2));
  }

  // return angle between u and v in degrees
  public static Rotation2d angleBetweenVectors(Translation2d u, Translation2d v) {
    double numerator = u.getX() * v.getX() + u.getY() * v.getY();
    double denominator = u.getNorm() * v.getNorm();
    return Rotation2d.fromRadians(Math.acos(numerator / denominator));
  }

  // return vector of proju->v
  public static Translation2d projectUonV(Translation2d u, Translation2d v) {
    double vMagnitude = v.getNorm();
    Translation2d vUnitVector = v.div(vMagnitude);
    double uDotVOverMagnitudeV = (u.getX() * v.getX() + u.getY() * v.getY()) / vMagnitude;
    return vUnitVector.times(uDotVOverMagnitudeV);
  }

  // returns 1 if p->q->r is CW and 2 if CCW
  public static int orientation(Translation2d p, Translation2d q, Translation2d r) {
    double val =
        (q.getY() - p.getY()) * (r.getX() - q.getX())
            - (q.getX() - p.getX()) * (r.getY() - q.getY());

    return (val > 0) ? 1 : 2;
  }

  // returns whether 2 line segments intersect each other
  public static boolean intersect(
      Translation2d start1, Translation2d end1, Translation2d start2, Translation2d end2) {
    int o1 = orientation(start1, end1, start2);
    int o2 = orientation(start1, end1, end2);
    int o3 = orientation(start2, end2, start1);
    int o4 = orientation(start2, end2, end1);

    return (o1 != o2 && o3 != o4);
  }
}
