// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class ArmConstants {
  // TODO: Fix these to comply to the mechanical ppls kg
  public static final int kArmMotorID = -1;
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
      new TrapezoidProfile.Constraints(0.45, 0.45);

  public static final double kArmToleranceAngle = Units.degreesToRadians(2);
  public static final double kArmToleranceAngularVelocity = Units.degreesToRadians(2);

  public static final int armID = 20;
  public static final double kArmS = 1;
  public static final double kArmG = 1;
  public static final double kArmV = 1;
  public static final double kArmA = 1;

  public static final double kArmHighPositionMeters = 1;
  public static final double kArmMidPositionMeters = 1;
  public static final double kArmLowPositionMeters = 1;
  public static final int kNumArmMotors = 1;
  public static final double kArmGearing = 1;
  public static final double jKgMetersSquared = 1;
  public static final double kArmLengthMeters = 1;
  public static final double minAngleRads = 0;
  public static final double maxAngleRads = Math.PI;
  public static final double armMassKg = 1;
  public static final double kArmCurrentThreshold = 1;
}
