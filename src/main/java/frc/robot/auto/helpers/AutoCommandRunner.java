// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.auto.AutoConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class AutoCommandRunner {
  private List<AutoCommandMarker> commandMarkers = new ArrayList<>();
  private List<AutoCommandMarker> startedCommandMarkers = new ArrayList<>();
  private Pose2d lastPose;

  public AutoCommandRunner(
      List<PathPlannerTrajectory.EventMarker> eventMarker, Map<String, Command> eventMap) {
    this.commandMarkers = convertEventMarker(eventMarker, eventMap);
  }

  private List<AutoCommandMarker> convertEventMarker(
      List<PathPlannerTrajectory.EventMarker> eventMarkers, Map<String, Command> eventMap) {
    List<AutoCommandMarker> convertedMarkers = new ArrayList<>();
    for (PathPlannerTrajectory.EventMarker eventMarker : eventMarkers) {
      for (String name : eventMarker.names) {
        if (!eventMap.containsKey(name)) continue;

        Command command = eventMap.get(name);
        AutoCommandMarker commandMarker =
            new AutoCommandMarker(eventMarker.positionMeters, command);
        convertedMarkers.add(commandMarker);
      }
    }
    return convertedMarkers;
  }

  public void execute(Pose2d currentPose) {
    for (int i = 0; i < commandMarkers.size(); i++) {
      AutoCommandMarker marker = commandMarkers.get(i);
      boolean atMarker =
          lastPose == null
              ? isAtMarker(marker.getPos(), currentPose)
              : isAtMarker(marker.getPos(), currentPose, lastPose);

      if (atMarker) {
        marker.getCommand().schedule();
        startedCommandMarkers.add(marker);
        commandMarkers.remove(i);
        i--;
      }
    }

    for (int i = 0; i < startedCommandMarkers.size(); i++) { // cancel started commands
      AutoCommandMarker marker = startedCommandMarkers.get(i);
      boolean atMarker =
          lastPose == null
              ? isAtMarker(marker.getPos(), currentPose)
              : isAtMarker(marker.getPos(), currentPose, lastPose);

      if (atMarker) {
        marker.getCommand().cancel();
        startedCommandMarkers.add(marker);
        commandMarkers.remove(i);
        i--;
      }
    }
    lastPose = currentPose;
  }

  public void end() {
    for (AutoCommandMarker marker : startedCommandMarkers) {
      marker.getCommand().cancel();
    }
  }

  private boolean isAtMarker(Translation2d marker, Pose2d currentPose) {
    if (marker == null || currentPose == null) {
      return false;
    }

    double distance = marker.getDistance(currentPose.getTranslation());

    return distance < kCommandMarkerThreshold;
  }

  private boolean isAtMarker(Translation2d marker, Pose2d currentPose, Pose2d lastPose) {
    if (marker == null || currentPose == null || lastPose == null) {
      return false;
    }

    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d lastTranslation = lastPose.getTranslation();

    double distanceBetweenTrajectoryPoses =
        Math.abs(currentTranslation.getDistance(lastTranslation)) * 6; // TODO: WTF
    double distanceBetweenLastPose = Math.abs(lastTranslation.getDistance(marker));
    double distanceBetweenCurrentPose = Math.abs(currentTranslation.getDistance(marker));

    // essentially finding if the marker is in between the 2 trajectories
    // by kinda creating an ellipse/box while allowing for some tolerance
    return distanceBetweenCurrentPose < distanceBetweenTrajectoryPoses
        && distanceBetweenLastPose < distanceBetweenTrajectoryPoses;
  }
}
