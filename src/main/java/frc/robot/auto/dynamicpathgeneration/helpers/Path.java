// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class Path {
  private List<Waypoint> waypoints;
  private int points;

  public Path(List<PathNode> pathNodes, Rotation2d startRotation, Rotation2d endRotation) {
    // convert path nodes to translations
    points = pathNodes.size();
    List<Translation2d> positions = new ArrayList<>();
    for (int i = 0; i < points; i++) {
      positions.add(pathNodes.get(i).getPoint());
    }
    // path length from node to final node, excluding passages
    double[] remainingPathLength = new double[points];
    remainingPathLength[points - 1] = 0;
    for (int i = points - 2; i >= 0; i--) {
      remainingPathLength[i] = remainingPathLength[i + 1];
      if (pathNodes.get(i).getType() != PathNode.NodeType.PASSAGE)
        remainingPathLength[i] += positions.get(i).getDistance(positions.get(i + 1));
    }
    // convert poses to waypoints
    waypoints = new ArrayList<>();
    for (int i = 0; i < points; i++) {
      // calculate holonomicAngle
      Rotation2d holonomicAngle;
      // first point case, use startRotation
      if (i == 0) {
        holonomicAngle = startRotation;
      } else {
        // total rotation to end
        Rotation2d prevRotation = waypoints.get(i - 1).getHolonomicAngle();
        Rotation2d totRotation = endRotation.minus(prevRotation);
        // rotation step size
        Rotation2d dRotation = totRotation.div(remainingPathLength[i - 1]);
        // marks the direction the robot is slowly turning to
        boolean ccw = dRotation.getDegrees() >= 0;
        // calculate holonomic angle
        holonomicAngle =
            prevRotation.plus(dRotation.times(remainingPathLength[i - 1] - remainingPathLength[i]));
        // passage case, convert to multiple of 90*
        if (pathNodes.get(i).getType() == PathNode.NodeType.PASSAGE) {
          double[] radLock = {-Math.PI, -Math.PI / 2, 0, Math.PI / 2, Math.PI};

          for (double rad : radLock) {
            if (ccw
                && rad - holonomicAngle.getRadians() < Math.PI / 2
                && rad - holonomicAngle.getRadians() >= 0) {
              holonomicAngle = new Rotation2d(rad);
            } else if (!ccw
                && holonomicAngle.getRadians() - rad < Math.PI / 2
                && holonomicAngle.getRadians() - rad >= 0) {
              holonomicAngle = new Rotation2d(rad);
            } else if (Math.abs(holonomicAngle.getRadians() - rad) < 0.1) {
              holonomicAngle = new Rotation2d(rad);
              break;
            }
          }
        }
      }

      // calculate anchorPoint, prevControl, nextControl
      Translation2d anchorPoint = positions.get(i);
      Translation2d prevControl;
      Translation2d nextControl;
      // passage case, use horizontal bezier controls
      if (pathNodes.get(i).getType() == PathNode.NodeType.PASSAGE) {
        // set prev control
        double prevControlPointScalar = kRegularControlPointScalar;
        if (i > 0 && pathNodes.get(i - 1).getType() == PathNode.NodeType.PASSAGE)
          prevControlPointScalar = kBetweenPassageControlPointScalar;
        Translation2d prevControlVector =
            new Translation2d(positions.get(i - 1).minus(positions.get(i)).getX(), 0);
        prevControl = prevControlVector.times(prevControlPointScalar).plus(anchorPoint);
        // set next control
        double nextControlPointScalar = kRegularControlPointScalar;
        if (i < points - 1 && pathNodes.get(i + 1).getType() == PathNode.NodeType.PASSAGE)
          nextControlPointScalar = kBetweenPassageControlPointScalar;
        Translation2d nextControlVector =
            new Translation2d(positions.get(i + 1).minus(positions.get(i)).getX(), 0);
        nextControl = nextControlVector.times(nextControlPointScalar).plus(anchorPoint);
      }
      // first point case, use point to point bezier control
      else if (i == 0) {
        prevControl = null;
        Translation2d thisPointToNextPoint =
            positions.get(i + 1).minus(positions.get(i)).times(kRegularControlPointScalar);

        nextControl = anchorPoint.plus(thisPointToNextPoint);
      }
      // last point case, use point to point bezier control
      else if (i == points - 1) {
        Translation2d thisPointToPrevPoint =
            positions.get(i - 1).minus(positions.get(i)).times(kRegularControlPointScalar);

        prevControl = anchorPoint.plus(thisPointToPrevPoint);
        nextControl = null;
      }
      // else use optimal bezier controls
      else {
        double prevControlPointScalar = kRegularControlPointScalar;
        double nextControlPointScalar = kRegularControlPointScalar;
        // apply tight control point scalar in the community zone to avoid hitting charging station
        if (pathNodes.get(i).getType() == PathNode.NodeType.PRESINK) {
          if (pathNodes.get(i - 1).getType() == PathNode.NodeType.PRESINK) {
            prevControlPointScalar = kBetweenPreSinkPointScalar;
          }
          if (pathNodes.get(i + 1).getType() == PathNode.NodeType.PRESINK) {
            nextControlPointScalar = kBetweenPreSinkPointScalar;
          }
        }

        Translation2d[] controlPoints =
            findControlPoints(
                positions.get(i - 1),
                positions.get(i),
                positions.get(i + 1),
                prevControlPointScalar,
                nextControlPointScalar);

        prevControl = controlPoints[0];
        nextControl = controlPoints[1];
      }
      waypoints.add(new Waypoint(anchorPoint, prevControl, nextControl, holonomicAngle));
    }
  }

  public List<Waypoint> getWaypoints() {
    return this.waypoints;
  }

  public JSONObject getJson() {
    JSONObject fullJson = new JSONObject();

    JSONArray pathJson = new JSONArray();
    for (Waypoint waypoint : waypoints) {
      pathJson.add(waypoint.getJson());
    }
    fullJson.put("waypoints", pathJson);

    JSONArray markerJson = new JSONArray();
    fullJson.put("markers", markerJson);

    return fullJson;
  }

  // find optimal bezier control points for a point given the point right before and the point right
  // after
  public Translation2d[] findControlPoints(
      Translation2d startPoint,
      Translation2d desiredPoint,
      Translation2d endPoint,
      double prevControlPointScalar,
      double nextControlPointScalar) {
    Translation2d desiredToStartVector = startPoint.minus(desiredPoint);
    Translation2d desiredToEndVector = endPoint.minus(desiredPoint);

    Rotation2d beta = GeometryUtil.angleBetweenVectorsCCW(desiredToStartVector, desiredToEndVector);
    Rotation2d alpha = Rotation2d.fromDegrees((180 - beta.getDegrees()) / 2);

    Translation2d desiredToStartTransformed = desiredToStartVector.rotateBy(alpha.unaryMinus());
    Translation2d projDesiredToStartOnTransform =
        GeometryUtil.projectUonV(desiredToStartVector, desiredToStartTransformed);
    Translation2d startPointControlPoint =
        desiredPoint.plus(projDesiredToStartOnTransform.times(prevControlPointScalar));

    Translation2d desiredToEndTransformed = desiredToEndVector.rotateBy(alpha);
    Translation2d projDesiredToEndOnTransform =
        GeometryUtil.projectUonV(desiredToEndVector, desiredToEndTransformed);
    Translation2d endPointControlPoint =
        desiredPoint.plus(projDesiredToEndOnTransform.times(nextControlPointScalar));

    return new Translation2d[] {startPointControlPoint, endPointControlPoint};
  }
}
