// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.limeLight.commands;

import static frc.robot.Constants.LimeLightConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class getLimeLightValues {

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
    if (!tv.exists()) return false;
    else if ((int) tv.getValue().getValue() == 1) return true;
    else if ((int) tv.getValue().getValue() == 0) return false;
    System.out.println("Something's wrong, here is the value " + tv.getValue().getValue());
    return false;
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
