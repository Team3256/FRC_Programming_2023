// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class ArmConstants {
  // TODO: Fix constants
  public static final int kArmMotorID = 10;
  public static final double kArmInertia =
      1; // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/SingleJointedArmSim.html#%3Cinit%3E(edu.wpi.first.math.system.plant.DCMotor,double,double,double,double,double,double,boolean)
  public static final double kMinAngleRads = 1;
  public static final double kMaxAngleRads = 1;
  public static final double kArmMassKg = 1;
  public static final boolean kArmSimGravity = true;

  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final TrapezoidProfile.Constraints kArmContraints =
      new TrapezoidProfile.Constraints(0.1, 0.1);

  public static final Rotation2d kArmToleranceAngle = new Rotation2d();
  public static final Rotation2d kArmToleranceAngularVelocity = new Rotation2d();

  public static final double kArmAngleConstraint = -12.881991;
  public static final double kArmS = 1;
  public static final double kArmG = 1;
  public static final double kArmV = 1;
  public static final double kArmA = 1;
  public static final int kNumArmMotors = 1;
  public static final double kArmGearing = 1;
  public static final double jKgMetersSquared = 1;
  public static final double kArmLengthMeters = 10;
  public static final double minAngleRads = 0;
  public static final double maxAngleRads = Math.PI;
  public static final double armMassKg = 1;
  public static final double kArmCurrentThreshold = 1;
}
