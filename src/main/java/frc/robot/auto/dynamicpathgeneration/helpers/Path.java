// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
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
    points = pathNodes.size();
    List<Translation2d> positions = new ArrayList<>();
    for (int i = 0; i < points; i++) {
      positions.add(pathNodes.get(i).getPoint());
    }
    // convert positions into poses
    List<Pose2d> poses = new ArrayList<>();
    for (Translation2d position : positions) {
      poses.add(new Pose2d(position, new Rotation2d(0)));
    }

    // dRotation, similar to dx or dy, representing a small change in rotation
    // throughout the path when it is not in narrow passage
    double[] pathLengthTo = new double[points];
    pathLengthTo[0] = 0;
    for (int i = 1; i < points; i++) {
      pathLengthTo[i] = pathLengthTo[i - 1];
      if (!pathNodes.get(i).isPassage()) {
        pathLengthTo[i] += positions.get(i).getDistance(positions.get(i - 1));
      }
    }
    Rotation2d totRotation = endRotation.minus(startRotation);
    if (totRotation.getRadians() < -Math.PI) totRotation.plus(new Rotation2d(2 * Math.PI));
    if (totRotation.getRadians() > Math.PI) totRotation.minus(new Rotation2d(2 * Math.PI));
    Rotation2d dRotation = totRotation.div(pathLengthTo[points - 1]);

    // convert poses to waypoints
    waypoints = new ArrayList<>();
    for (int i = 0; i < points; i++) {
      Translation2d anchorPoint = poses.get(i).getTranslation();
      // get positive holonomic
      Rotation2d holonomicAngle = startRotation.plus(dRotation.times(pathLengthTo[i]));
      if (holonomicAngle.getDegrees() < 0)
        holonomicAngle = holonomicAngle.plus(new Rotation2d(2 * Math.PI));
      Translation2d prevControl;
      Translation2d nextControl;
      // horizontal passage case: horizontal proj scale and lock angle compass NESW
      if (pathNodes.get(i).isPassage()) {
        Translation2d baseL =
            new Translation2d(poses.get(i - 1).minus(poses.get(i)).getTranslation().getX(), 0);
        prevControl = baseL.times(kControlPointScalar).plus(anchorPoint);
        Translation2d baseR =
            new Translation2d(poses.get(i + 1).minus(poses.get(i)).getTranslation().getX(), 0);
        nextControl = baseR.times(kControlPointScalar).plus(anchorPoint);
        double[] radLock = {0, Math.PI / 2, Math.PI, 3 * Math.PI / 2, 2 * Math.PI};
        for (double rad : radLock) {
          if (Math.abs(holonomicAngle.getRadians() - rad) <= Math.PI / 4) {
            holonomicAngle = new Rotation2d(rad);
          }
        }
      } else if (i == 0) {
        prevControl = null;
        Translation2d thisPointToNextPoint =
            positions.get(i + 1).minus(positions.get(i)).times(kControlPointScalar);

        nextControl = anchorPoint.plus(thisPointToNextPoint);
      } else if (i == points - 1) {
        Translation2d thisPointToPrevPoint =
            positions.get(i - 1).minus(positions.get(i)).times(kControlPointScalar);

        prevControl = anchorPoint.plus(thisPointToPrevPoint);
        nextControl = null;
      } else {
        Translation2d[] controlPoints =
            findControlPoints(positions.get(i - 1), positions.get(i), positions.get(i + 1));

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

  public Translation2d[] findControlPoints(
      Translation2d startPoint, Translation2d desiredPoint, Translation2d endPoint) {
    Translation2d desiredToStartVector = startPoint.minus(desiredPoint);
    Translation2d desiredToEndVector = endPoint.minus(desiredPoint);

    Rotation2d beta = GeometryUtil.angleBetweenVectorsCCW(desiredToStartVector, desiredToEndVector);
    Rotation2d alpha = Rotation2d.fromDegrees((180 - beta.getDegrees()) / 2);

    Translation2d desiredToStartTransformed = desiredToStartVector.rotateBy(alpha.unaryMinus());
    Translation2d projDesiredToStartOnTransform =
        GeometryUtil.projectUonV(desiredToStartVector, desiredToStartTransformed);
    Translation2d startPointControlPoint =
        desiredPoint.plus(projDesiredToStartOnTransform.times(kControlPointScalar));

    Translation2d desiredToEndTransformed = desiredToEndVector.rotateBy(alpha);
    Translation2d projDesiredToEndOnTransform =
        GeometryUtil.projectUonV(desiredToEndVector, desiredToEndTransformed);
    Translation2d endPointControlPoint =
        desiredPoint.plus(projDesiredToEndOnTransform.times(kControlPointScalar));

    return new Translation2d[] {startPointControlPoint, endPointControlPoint};
  }
}
