// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import static frc.robot.Constants.FieldConstants;
import static frc.robot.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionWrapper {
  private PhotonCamera camera;
  private AprilTagFieldLayout fieldLayout;
  private RobotPoseEstimator photonPoseEstimator;

  ArrayList<AprilTag> tagList = new ArrayList<AprilTag>(FieldConstants.aprilTags.values());

  public PhotonVisionWrapper() {
    camera = new PhotonCamera(VisionConstants.kCameraName);
    fieldLayout =
        new AprilTagFieldLayout(tagList, FieldConstants.fieldLength, FieldConstants.fieldWidth);
    ArrayList<Pair<PhotonCamera, Transform3d>> cameraList = new ArrayList<>();
    cameraList.add(new Pair<>(camera, VisionConstants.robotToCam));

    // current strategy based on the april tag that is closest to the reference pose
    photonPoseEstimator =
        new RobotPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraList);
  }

  /*
   * returns the camera last result for april tag targets, which can be used
   * for target processing and alignment purposes
   * @return the last PhotonPipelineResult from the camera for the timestamp
   */
  public PhotonPipelineResult getLastResult() {
    return camera.getLatestResult();
  }

  /*
   * Method used for providing swerve pose estimator a vision measurement
   * @param prevEstimatedRobotPose the last calculated pose from the poseEstimator
   * @return the last PhotonPipelineResult from the camera for the timestamp
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
