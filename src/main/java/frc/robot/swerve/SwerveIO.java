// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.swerve.helpers.SwerveModule;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    public double[] ypr = new double[3];
    public double frontRightModuleVelocity = 0.0;
    public double frontLeftModuleVelocity = 0.0;
    public double backLeftModuleVelocity = 0.0;
    public double backRightModuleVelocity = 0.0;

    public double frontRightModuleAngle = 0.0;
    public double frontLeftModuleAngle = 0.0;
    public double backLeftModuleAngle = 0.0;
    public double backRightModuleAngle = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {}

  public default Rotation2d getYaw() {
    return new Rotation2d();
  }

  public default void zeroGyro() {}

  public default void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {}

  /* Used by SwerveControllerCommand in Auto */
  public default void setModuleStates(SwerveModuleState[] desiredStates) {}

  public default SwerveModule[] getSwerveModules() {
    return new SwerveModule[] {};
  }
}
