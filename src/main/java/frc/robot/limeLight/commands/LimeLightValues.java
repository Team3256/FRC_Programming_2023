package frc.robot.limelight.commands;// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LimeLightConstants.*;
 import static frc.robot.Constants.goalHeightInches;
 import static frc.robot.Constants.mountingHeight;
 import static frc.robot.Constants.limelightMountAngleDegrees;
 import static frc.robot.Constants.limelightLensHeightInches;
 import static frc.robot.Constants.angleToGoalRadians;

public class LimeLightValues {
  private static int users = 0;

  private static NetworkTable limelight;

  static {
//        double[] rawDistance = new double[LIMELIGHT_TUNED_DATA.size()];
//        double[] actualDistance = new double[LIMELIGHT_TUNED_DATA.size()];
//        for(int i = 0; i < LIMELIGHT_TUNED_DATA.size(); i++) {
//            int[] data = LIMELIGHT_TUNED_DATA.get(i);
//            rawDistance[i] = data[0];
//            actualDistance[i] = data[1];
//        }
//
//        tunedDistance = new SplineInterpolator().interpolate(rawDistance, actualDistance);
  }

  //Doesn't Allow Instancing
  private LimeLightValues(){}

  public static void init() {
    //Setting up NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    limelight = inst.getTable("limelight");

    //Setting up default stream
    limelight.getEntry("ledMode").setNumber(1); //Forces the LED Mode to off
    limelight.getEntry("camMode").setNumber(0); //Uses Vision Processor Mode
    limelight.getEntry("pipeline").setNumber(0); //Uses pipeline #0
    limelight.getEntry("stream").setNumber(2); //Driver Camera Main, Vision Camera Lower-Right Corner
    limelight.getEntry("snapshot").setNumber(0); //Takes no snapshots

    if(getLimelightValue("tx").getDouble(1000) == 1000)
      System.out.println("Limelight not responding");
  }
  /**
   * @param value
   * @return entry with name of value
   */
  private static NetworkTableEntry getLimelightValue(String value){
    if (limelight == null) {
      System.out.println("Limelight not Initialized! Returning Bad NetworkTable!");
      return new NetworkTableEntry(NetworkTableInstance.getDefault(), 0);
    }
    return limelight.getEntry(value);
  }
  /**
   * @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   */
  public static double getTx(){
    return getLimelightValue("tx").getDouble(0);
  }
  /**
   * @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   */
  public static double getTy(){
    return getLimelightValue("ty").getDouble(0);
  }
  /**
   * @return Target Area (0% of image to 100% of image)
   */
  public static double getTa(){
    return getLimelightValue("ta").getDouble(0);
  }
  /**
   * @return Skew or rotation (-90 degrees to 0 degrees)
   */
  public static double getTs(){
    return getLimelightValue("ts").getDouble(0);
  }
  /**
   * @return Whether the limelight has any valid targets (0 or 1)
   */
  public static double getTv(){
    return getLimelightValue("tv").getDouble(0);
  }
  /**
   * @return Number array of corner coordinates [x0,x1,etc]
   */
  public static double[] getTcornx(){
    return getLimelightValue("tcornx").getDoubleArray(new double[4]);
  }
  /**
   * @return Number array of corner coordinates [y0,y1,etc]
   */
  public static double[] getTcorny(){
    return getLimelightValue("tcorny").getDoubleArray(new double[4]);
  }
  /**
   * @return raw distance to target (inches)
   */
  public static double getRawDistanceToTarget(){
    return (goalHeightInches - mountingHeight)/Math.tan(toRadians(limelightMountAngleDegrees + getTy()));
  }

  /**
   * @return tuned distance to target (inches)
   */
  public static double getTunedDistanceToTarget(){
    double rawDistance = getRawDistanceToTarget();
    return rawDistance;
//        try {
//            return tunedDistance.value(rawDistance);
//        } catch (Exception e) {
//            logger.warning("Distance from Limelight is out of range of interpolating");
//            return rawDistance;
//        }
  }

  public static boolean isTargetDetected(){
    if(getTv() == 1){
      return true;
    }
    else{
      return false;
    }
  }

  /**
   * @param degrees
   * @return radians
   */
  public static double toRadians(double degrees){
    return degrees * Math.PI/180.0;
  }

  /**
   *  Methods that enable and disable the limelight
   */
  public static void disable(){
    if (--users == 0) {
      LimeLightValues.getLimelightValue("ledMode").setNumber(1);
    }
  }

  /**
   * Enables the LEDs on the limelight, LEDs should be on when limelight is in use.
   */
  public static void enable(){
    users++;
    LimeLightValues.getLimelightValue("ledMode").setNumber(3);
  }

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static NetworkTableEntry tx = table.getEntry("tx");
  private static NetworkTableEntry ty = table.getEntry("ty");
  private static NetworkTableEntry ta = table.getEntry("ta");
  private static NetworkTableEntry tv = table.getEntry("tv");
  private static DoubleArraySubscriber botpose =
          table.getDoubleArrayTopic("botpose").subscribe(new double[] {});

  private static IntegerSubscriber tclass = table.getIntegerTopic("tclass").subscribe(0);

  //
  static double x = tx.getDouble(0.0);
  static double y = ty.getDouble(0.0);
  static double area = ta.getDouble(0.0);

  static Pose3d pose = new Pose3d();
  private static Translation3d mTranslation = new Translation3d();
  private static Rotation3d mRotation = new Rotation3d();

  public static void main(String[] args) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public static boolean targetValidity() {
    return (!tv.exists())? false: ((int) tv.getValue().getValue() == 1)? true:false;

  }

  public static double camTran() {
    return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  public static Pose3d botPose() {

    double[] poseArray = botpose.get();

    Translation3d translation = new Translation3d(poseArray[0], poseArray[1], 0.0);
    Rotation3d rotation = new Rotation3d(poseArray[3], poseArray[4], poseArray[5]);

    return new Pose3d(translation, rotation);
  }

  public static Pose3d camPose() {

    double[] poseArray = botpose.get();
    Translation3d translation =
            new Translation3d(poseArray[0], poseArray[1], DIST_FROM_BOTTOM_OF_ROBOT_TO_CAMERA);
    Rotation3d rotation = new Rotation3d(poseArray[3], poseArray[4], poseArray[5]);

    return new Pose3d(translation, rotation);
  }

  public static IntegerSubscriber TClass() {

    return tclass;
  }

}