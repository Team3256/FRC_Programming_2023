// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class WayPoint {
  Translation2d anchorPoint;
  Translation2d prevControl;
  Translation2d nextControl;
  Rotation2d holonomicAngle;

  WayPoint() {}

  WayPoint(
      Translation2d anchorPoint,
      Translation2d prevControl,
      Translation2d nextControl,
      Rotation2d holonomicAngle) {
    this.anchorPoint = anchorPoint;
    this.prevControl = prevControl;
    this.nextControl = nextControl;

    this.holonomicAngle = holonomicAngle;
  }

  public JSONObject getJson() {
    JSONObject ret = new JSONObject();
    // we set
    ret.put("anchorPoint", getTransJson(anchorPoint));
    ret.put("prevControl", getTransJson(prevControl));
    ret.put("nextControl", getTransJson(nextControl));
    ret.put("holonomicAngle", holonomicAngle.getDegrees());
    // default
    ret.put("isReversal", false);
    ret.put("velOverride", null);
    ret.put("isLocked", false);
    ret.put("isStopPoint", false);
    ret.put("stopEvent", getEventJson());
    return ret;
  }

  public JSONObject getEventJson() {
    JSONObject ret = new JSONObject();
    ret.put("names", new JSONArray());
    ret.put("executionBehavior", "parallel");
    ret.put("waitBehavior", "none");
    ret.put("waitTime", 0);
    return ret;
  }

  public JSONObject getTransJson(Translation2d point) {
    if (point == null) return null;
    JSONObject ret = new JSONObject();
    ret.put("x", point.getX());
    ret.put("y", point.getY());
    return ret;
  }
}
