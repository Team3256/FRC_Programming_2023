// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.limelight;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

public class Limelight {

  private static ObjectMapper mapper;

  /** Print JSON Parse time to the console in milliseconds */
  static boolean profileJSON = false;

  static String sanitizeName(String name) {
    if (name.equals("")) {
      return "limelight";
    }
    return name;
  }

  public static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  public static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  public static String getLimelightNTString(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getString("");
  }

  public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    URL url;
    try {
      url = new URL(urlString);
      return url;
    } catch (MalformedURLException e) {
      System.err.println("bad LL URL");
    }
    return null;
  }
  /////
  /////

  public static double getTX(String limelightName) {
    return getLimelightNTDouble(limelightName, "tx");
  }

  public static double getTY(String limelightName) {
    return getLimelightNTDouble(limelightName, "ty");
  }

  public static double getTA(String limelightName) {
    return getLimelightNTDouble(limelightName, "ta");
  }

  public static double getLatency_Pipeline(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl");
  }

  public static double getLatency_Capture(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl_cap");
  }

  public static double getCurrentPipelineIndex(String limelightName) {
    return getLimelightNTDouble(limelightName, "getpipe");
  }

  public static String getJSONDump(String limelightName) {
    return getLimelightNTString(limelightName, "json");
  }

  public static double[] getBotpose(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  public static double[] getBotpose_wpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  public static double[] getBotpose_wpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  public static double[] getBotPose_TargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_targetSpace");
  }

  public static double[] getCameraPose_TargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
  }

  public static double[] getTargetPose_CameraSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
  }

  public static double[] getTargetPose_RobotSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
  }

  public static double[] getTargetColor(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "tc");
  }

  public static double getFiducialID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tid");
  }

  public static String getNeuralClassID(String limelightName) {
    return getLimelightNTString(limelightName, "tclass");
  }

  public static boolean isConeDetected(String limelightName) {
    return getNeuralClassID(limelightName).equals("cone");
  }

  public static boolean isCubeDetected(String limelightName) {
    return getNeuralClassID(limelightName).equals("cube");
  }

  public static void setPipelineIndex(String limelightName, int pipelineIndex) {
    setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
  }

  /** The LEDs will be controlled by Limelight pipeline settings, and not by robot code. */
  public static void setLEDMode_PipelineControl(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 0);
  }

  public static void setLEDMode_ForceOff(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 1);
  }

  public static void setLEDMode_ForceBlink(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 2);
  }

  public static void setLEDMode_ForceOn(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 3);
  }

  public static void setStreamMode_Standard(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 0);
  }

  public static void setStreamMode_PiPMain(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 1);
  }

  public static void setStreamMode_PiPSecondary(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 2);
  }

  /**
   * Sets the crop window. The crop window in the UI must be completely open for dynamic cropping to
   * work.
   */
  public static void setCropWindow(
      String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double[] entries = new double[4];
    entries[0] = cropXMin;
    entries[1] = cropXMax;
    entries[2] = cropYMin;
    entries[3] = cropYMax;
    setLimelightNTDoubleArray(limelightName, "crop", entries);
  }

  public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
    setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
  }

  public static double[] getPythonScriptData(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  /** Asynchronously take snapshot. */
  public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
    return CompletableFuture.supplyAsync(
        () -> {
          return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
  }

  private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
    URL url = getLimelightURLString(tableName, "capturesnapshot");
    try {
      HttpURLConnection connection = (HttpURLConnection) url.openConnection();
      connection.setRequestMethod("GET");
      if (snapshotName != null && snapshotName != "") {
        connection.setRequestProperty("snapname", snapshotName);
      }

      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        return true;
      } else {
        System.err.println("Bad LL Request");
      }
    } catch (IOException e) {
      System.err.println(e.getMessage());
    }
    return false;
  }

  /** Parses Limelight's JSON results dump into a LimelightResults Object */
  public static LimelightResults getLatestResults(String limelightName) {

    long start = System.nanoTime();
    LimelightResults results = new LimelightResults();
    if (mapper == null) {
      mapper =
          new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }

    try {
      results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
    } catch (JsonProcessingException e) {
      System.err.println("lljson error: " + e.getMessage());
    }

    long end = System.nanoTime();
    double millis = (end - start) * .000001;
    results.targetingResults.latency_jsonParse = millis;
    if (profileJSON) {
      System.out.printf("lljson: %.2f\r\n", millis);
    }

    return results;
  }

  public static boolean hasValidTargets(String limelightName) {
    return getLimelightNTTableEntry(limelightName, "tv").getDouble(0) == 1;
  }
}
