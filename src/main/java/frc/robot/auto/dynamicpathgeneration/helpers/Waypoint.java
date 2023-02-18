// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class Waypoint {
  private Translation2d anchorPoint;
  private Translation2d prevControl;
  private Translation2d nextControl;
  private Rotation2d holonomicAngle;
  private Rotation2d heading;
  public boolean forceHorizontal;

  Waypoint(
      Translation2d anchorPoint,
      Translation2d prevControl,
      Translation2d nextControl,
      Rotation2d holonomicAngle) {
    this.anchorPoint = anchorPoint;
    this.prevControl = prevControl;
    this.nextControl = nextControl;
    this.holonomicAngle = holonomicAngle;

    if (nextControl != null) {
      this.heading = nextControl.minus(anchorPoint).getAngle();
    } else if (prevControl != null) {
      this.heading = anchorPoint.minus(prevControl).getAngle();
    } else {
      this.heading = new Rotation2d();
    }
  }

  public JSONObject getJson() {
    JSONObject json = new JSONObject();
    json.put("anchorPoint", createJSONFromTranslation(anchorPoint));
    json.put("prevControl", createJSONFromTranslation(prevControl));
    json.put("nextControl", createJSONFromTranslation(nextControl));
    json.put("holonomicAngle", holonomicAngle.getDegrees());
    // default
    json.put("isReversal", false);
    json.put("velOverride", null);
    json.put("isLocked", false);
    json.put("isStopPoint", false);
    json.put("stopEvent", getEventJson());
    return json;
  }

  private JSONObject getEventJson() {
    JSONObject json = new JSONObject();
    json.put("names", new JSONArray());
    json.put("executionBehavior", "parallel");
    json.put("waitBehavior", "none");
    json.put("waitTime", 0);
    return json;
  }

  private JSONObject createJSONFromTranslation(Translation2d point) {
    if (point == null) return null;

    JSONObject json = new JSONObject();
    json.put("x", point.getX());
    json.put("y", point.getY());
    return json;
  }

  public PathPoint waypointToPathPoint() {
    PathPoint pathPoint = new PathPoint(anchorPoint, heading, holonomicAngle);
    if (prevControl != null) {
      Translation2d anchorPrevVector = anchorPoint.minus(prevControl);
      pathPoint.withNextControlLength(anchorPrevVector.getNorm());
    }
    if (nextControl != null) {
      Translation2d anchorNextVector = anchorPoint.minus(nextControl);
      pathPoint.withNextControlLength(anchorNextVector.getNorm());
    }
    return pathPoint;
  }
}
