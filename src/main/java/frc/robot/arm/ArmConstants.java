// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

public final class ArmConstants {
  // TODO: Fix these to comply to the mechanical ppls kg
  public static final int ARM_MOTOR_ID = -1;
  public static final double kArmGearing = 1;
  public static final double kArmInertia =
      1; // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/SingleJointedArmSim.html#%3Cinit%3E(edu.wpi.first.math.system.plant.DCMotor,double,double,double,double,double,double,boolean)
  public static final double kArmLengthMeters = 1;
  public static final double kMinAngleRads = 1;
  public static final double kMaxAngleRads = 1;
  public static final double kArmMassKg = 1;
  public static final boolean kArmSimGravity = true;

  public static final double kP = 1;
  public static final double kI = 1;
  public static final double kD = 1;
  public static final double kFF = 1;
}
