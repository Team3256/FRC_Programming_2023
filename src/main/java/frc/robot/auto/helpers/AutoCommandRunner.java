// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class AutoCommandRunner {
  private List<AutoCommandMarker> commandMarkers = new ArrayList<>();
  private List<AutoCommandMarker> startedCommandMarkers = new ArrayList<>();
  private Pose2d lastPose;

  public AutoCommandRunner(List<AutoCommandMarker> markers) {
    commandMarkers = new ArrayList<>(markers);
  }

  public void execute(Pose2d currentPose) {
    if (lastPose == null) {
      for (int i = 0; i < commandMarkers.size(); i++) {
        AutoCommandMarker autoCommandMarker = commandMarkers.get(i);
        //                if (isAtMarker(autoCommandMarker.getMarker(), currentPose)) {
        //                    autoCommandMarker.getCommand().schedule();
        //                    startedCommandMarkers.add(autoCommandMarker);
        //                    commandMarkers.remove(i);
        //                    i--;
        //                }
      }

      for (int i = 0; i < startedCommandMarkers.size(); i++) { // cancel started commands
        AutoCommandMarker autoCommandMarker = startedCommandMarkers.get(i);
        autoCommandMarker.getCommand().cancel();
        startedCommandMarkers.remove(i);
        i--;
      }
    }
    //        } else {
    //            for (int i = 0; i < commandMarkers.size(); i++) {
    //                AutoCommandMarker autoCommandMarker = commandMarkers.get(i);
    ////                if (isAtMarker(autoCommandMarker.getMarker(), currentPose, lastPose)) {
    ////                    autoCommandMarker.getCommand().schedule();
    ////                    startedCommandMarkers.add(autoCommandMarker);
    ////                    commandMarkers.remove(i);
    ////                    i--;
    ////                }
    //            }

    for (int i = 0; i < startedCommandMarkers.size(); i++) { // cancel started commands
      AutoCommandMarker autoCommandMarker = startedCommandMarkers.get(i);
      //                if (isAtMarker(autoCommandMarker.getEndingMarker(), currentPose, lastPose))
      // {
      //                    autoCommandMarker.getCommand().cancel();
      //                    startedCommandMarkers.remove(i);
      //                    i--;
      //                }
    }
  }
  //        lastPose = currentPose;

  public void end() {
    Iterator<AutoCommandMarker> startedCommandMarkerIterator = startedCommandMarkers.iterator();
    while (startedCommandMarkerIterator.hasNext()) {
      startedCommandMarkerIterator.next().getCommand().cancel();
    }
  }

  //    private boolean isAtMarker(Translation2d marker, Pose2d currentPose) {
  //        if (marker == null || currentPose == null) {
  //            return false;
  //        }
  //
  //        double distance = marker.getDistance(currentPose.getTranslation());
  //        return distance < COMMAND_MARKER_THRESHOLD;
  //    }

  private boolean isAtMarker(Translation2d marker, Pose2d currentPose, Pose2d lastPose) {
    if (marker == null || currentPose == null || lastPose == null) {
      return false;
    }

    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d lastTranslation = lastPose.getTranslation();

    double distanceBetweenTrajectoryPoses =
        Math.abs(currentTranslation.getDistance(lastTranslation)) * 6;
    double distanceBetweenLastPose = Math.abs(lastTranslation.getDistance(marker));
    double distanceBetweenCurrentPose = Math.abs(currentTranslation.getDistance(marker));

    //        logger.info("Distance Between Trajectory Poses: " + distanceBetweenTrajectoryPoses);
    //        logger.info("Distance Between Last Pose: " + distanceBetweenLastPose);
    //        logger.info("Distance Between Current Pose: " + distanceBetweenCurrentPose);

    // essentially finding if the marker is in between the 2 trajectories
    // by kinda creating an ellipse/box while allowing for some tolerance
    return distanceBetweenCurrentPose < distanceBetweenTrajectoryPoses
        && distanceBetweenLastPose < distanceBetweenTrajectoryPoses;
  }
}
