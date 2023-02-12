// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.*;

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

  public Path(List<Translation2d> positions, Rotation2d startRotation, Rotation2d endRotation) {
    points = positions.size();
    // convert positions into poses
    List<Pose2d> poses = new ArrayList<>();
    for (Translation2d position : positions) {
      poses.add(new Pose2d(position, new Rotation2d(0)));
    }
    Pose2d startPose = poses.get(0);
    Pose2d endPose = poses.get(points - 1);

    // dRotation, similar to dx or dy, representing a small change in rotation
    // throughout the path
    Rotation2d dRotation = endPose.getRotation().minus(startPose.getRotation()).div(points);

    // convert poses to waypoints
    waypoints = new ArrayList<>();
    for (int i = 0; i < points; i++) {
      Translation2d anchorPoint = poses.get(i).getTranslation();
      Rotation2d holonomicAngle = startPose.getRotation().plus(dRotation.times(i));

      Translation2d prevControl;
      Translation2d nextControl;
      if (i == 0) {
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
