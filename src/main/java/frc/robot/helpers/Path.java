// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class Path {
  public List<Waypoint> waypoints;
  int length;
  double kControlPointScalar = 0.3;

  public Path(List<Pose2d> poses) {
    length = poses.size();
    Pose2d start = poses.get(0);
    Pose2d end = poses.get(length - 1);
    Rotation2d dRotation = end.getRotation().minus(start.getRotation()).div(length);
    waypoints = new ArrayList<>();
    for (int i = 0; i < length; i++) {
      // get anchor
      Translation2d anchorPoint = poses.get(i).getTranslation();
      // get angle
      Rotation2d holonomicAngle = start.getRotation().plus(dRotation.times(i));
      // set control
      Translation2d prevControl;
      Translation2d nextControl;
      if (i == 0) {
        prevControl = null;
        nextControl =
            poses
                .get(i)
                .getTranslation()
                .plus(
                    poses
                        .get(i + 1)
                        .getTranslation()
                        .minus(poses.get(i).getTranslation())
                        .times(kControlPointScalar));
      } else if (i == length - 1) {
        prevControl =
            poses
                .get(i)
                .getTranslation()
                .plus(
                    poses
                        .get(i - 1)
                        .getTranslation()
                        .minus(poses.get(i).getTranslation())
                        .times(kControlPointScalar));
        nextControl = null;
      } else {
        Translation2d[] controlPoints =
            findControlPoints(
                poses.get(i - 1).getTranslation(),
                poses.get(i).getTranslation(),
                poses.get(i + 1).getTranslation());
        prevControl = controlPoints[0];
        nextControl = controlPoints[1];
      }
      waypoints.add(new Waypoint(anchorPoint,prevControl,nextControl,holonomicAngle));
    }
  }

  public JSONObject getJson() {
    JSONObject ret = new JSONObject();

    // waypoints
    JSONArray pathJson = new JSONArray();
    for (int poseIndex = 0; poseIndex < length; poseIndex++) {
      pathJson.add(waypoints.get(poseIndex).getJson());
    }
    ret.put("waypoints", pathJson);

    // markers
    JSONArray markerJson = new JSONArray();
    ret.put("markers", markerJson);

    // ret
    return ret;
  }

  public Translation2d[] findControlPoints(
      Translation2d startPoint, Translation2d desiredPoint, Translation2d endPoint) {
    Translation2d desiredToStartVector = startPoint.minus(desiredPoint);
    Translation2d desiredToEndVector = endPoint.minus(desiredPoint);

    Rotation2d beta = TransHelper.angleBetweenVectorsCCW(desiredToStartVector, desiredToEndVector);
    Rotation2d alpha = Rotation2d.fromDegrees((180 - beta.getDegrees()) / 2);
    System.out.println("beta:" + beta.getDegrees());
    System.out.println("alpha:" + alpha.getDegrees());

    Translation2d desiredToStartTransformed = desiredToStartVector.rotateBy(alpha.unaryMinus());
    Translation2d projDesiredToStartOnTransform =
        TransHelper.projectUonV(desiredToStartVector, desiredToStartTransformed);
    Translation2d startPointControlPoint =
        desiredPoint.plus(projDesiredToStartOnTransform.times(kControlPointScalar));

    Translation2d desiredToEndTransformed = desiredToEndVector.rotateBy(alpha);
    Translation2d projDesiredToEndOnTransform =
        TransHelper.projectUonV(desiredToEndVector, desiredToEndTransformed);
    Translation2d endPointControlPoint =
        desiredPoint.plus(projDesiredToEndOnTransform.times(kControlPointScalar));

    return new Translation2d[] {startPointControlPoint, endPointControlPoint};
  }
}
