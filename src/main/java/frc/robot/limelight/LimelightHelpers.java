// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.limelight;

// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import com.fasterxml.jackson.annotation.JsonProperty;

class LimelightTarget_Retro {

  @JsonProperty("t6c_ts")
  double[] cameraPose_TargetSpace;

  @JsonProperty("t6r_fs")
  double[] robotPose_FieldSpace;

  @JsonProperty("t6r_ts")
  double[] robotPose_TargetSpace;

  @JsonProperty("t6t_cs")
  double[] targetPose_CameraSpace;

  @JsonProperty("t6t_rs")
  double[] targetPose_RobotSpace;

  @JsonProperty("ta")
  double ta;

  @JsonProperty("tx")
  double tx;

  @JsonProperty("txp")
  double tx_pixels;

  @JsonProperty("ty")
  double ty;

  @JsonProperty("typ")
  double ty_pixels;

  @JsonProperty("ts")
  double ts;

  LimelightTarget_Retro() {
    cameraPose_TargetSpace = new double[6];
    robotPose_FieldSpace = new double[6];
    robotPose_TargetSpace = new double[6];
    targetPose_CameraSpace = new double[6];
    targetPose_RobotSpace = new double[6];
  }
}

class LimelightTarget_Fiducial {

  @JsonProperty("fID")
  double fiducialID;

  @JsonProperty("fam")
  String fiducialFamily;

  @JsonProperty("t6c_ts")
  double[] cameraPose_TargetSpace;

  @JsonProperty("t6r_fs")
  double[] robotPose_FieldSpace;

  @JsonProperty("t6r_ts")
  double[] robotPose_TargetSpace;

  @JsonProperty("t6t_cs")
  double[] targetPose_CameraSpace;

  @JsonProperty("t6t_rs")
  double[] targetPose_RobotSpace;

  @JsonProperty("ta")
  double ta;

  @JsonProperty("tx")
  double tx;

  @JsonProperty("txp")
  double tx_pixels;

  @JsonProperty("ty")
  double ty;

  @JsonProperty("typ")
  double ty_pixels;

  @JsonProperty("ts")
  double ts;

  LimelightTarget_Fiducial() {
    cameraPose_TargetSpace = new double[6];
    robotPose_FieldSpace = new double[6];
    robotPose_TargetSpace = new double[6];
    targetPose_CameraSpace = new double[6];
    targetPose_RobotSpace = new double[6];
  }
}

class LimelightTarget_Barcode {}

class LimelightTarget_Classifier {

  @JsonProperty("class")
  String className;

  @JsonProperty("classID")
  double classID;

  @JsonProperty("conf")
  double confidence;

  @JsonProperty("zone")
  double zone;

  @JsonProperty("tx")
  double tx;

  @JsonProperty("txp")
  double tx_pixels;

  @JsonProperty("ty")
  double ty;

  @JsonProperty("typ")
  double ty_pixels;

  LimelightTarget_Classifier() {}
}

class LimelightTarget_Detector {

  @JsonProperty("class")
  String className;

  @JsonProperty("classID")
  double classID;

  @JsonProperty("conf")
  double confidence;

  @JsonProperty("ta")
  double ta;

  @JsonProperty("tx")
  double tx;

  @JsonProperty("txp")
  double tx_pixels;

  @JsonProperty("ty")
  double ty;

  @JsonProperty("typ")
  double ty_pixels;

  LimelightTarget_Detector() {}
}

class Results {

  @JsonProperty("pID")
  double pipelineID;

  @JsonProperty("tl")
  double latency_pipeline;

  @JsonProperty("tl_cap")
  double latency_capture;

  double latency_jsonParse;

  @JsonProperty("ts")
  double timestamp_LIMELIGHT_publish;

  @JsonProperty("ts_rio")
  double timestamp_RIOFPGA_capture;

  @JsonProperty("v")
  double valid;

  @JsonProperty("botpose")
  double[] botpose;

  @JsonProperty("botpose_wpired")
  double[] botpose_wpired;

  @JsonProperty("botpose_wpiblue")
  double[] botpose_wpiblue;

  @JsonProperty("Retro")
  LimelightTarget_Retro[] targets_Retro;

  @JsonProperty("Fiducial")
  LimelightTarget_Fiducial[] targets_Fiducials;

  @JsonProperty("Classifier")
  LimelightTarget_Classifier[] targets_Classifier;

  @JsonProperty("Detector")
  LimelightTarget_Detector[] targets_Detector;

  @JsonProperty("Barcode")
  LimelightTarget_Barcode[] targets_Barcode;

  Results() {
    botpose = new double[6];
    botpose_wpired = new double[6];
    botpose_wpiblue = new double[6];
    targets_Retro = new LimelightTarget_Retro[0];
    targets_Fiducials = new LimelightTarget_Fiducial[0];
    targets_Classifier = new LimelightTarget_Classifier[0];
    targets_Detector = new LimelightTarget_Detector[0];
    targets_Barcode = new LimelightTarget_Barcode[0];
  }
}

class LimelightResults {
  @JsonProperty("Results")
  Results targetingResults;

  LimelightResults() {
    targetingResults = new Results();
  }
}
