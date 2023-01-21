// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.swerve.helpers.SwerveModule;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModuleIO implements SwerveIO {

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveModule[] swerveModules;

  private final PigeonIMU gyro;

  private double[] ypr = new double[3];

  public SwerveModuleIO() {
    frontLeftModule = new SwerveModule(0, FrontLeft.constants);
    frontRightModule = new SwerveModule(1, FrontRight.constants);
    backLeftModule = new SwerveModule(2, BackLeft.constants);
    backRightModule = new SwerveModule(3, BackRight.constants);
    gyro = new PigeonIMU(pigeonID);
    gyro.configFactoryDefault();
    swerveModules =
        new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    inputs.frontRightModuleVelocity = frontRightModule.getModuleVelocity();
    inputs.frontLeftModuleVelocity = frontLeftModule.getModuleVelocity();
    inputs.backLeftModuleVelocity = backLeftModule.getModuleVelocity();
    inputs.backRightModuleVelocity = backRightModule.getModuleVelocity();

    inputs.frontRightModuleAngle = frontRightModule.getModuleAngle();
    inputs.frontLeftModuleAngle = frontLeftModule.getModuleAngle();
    inputs.backLeftModuleAngle = backLeftModule.getModuleAngle();
    inputs.backRightModuleAngle = backRightModule.getModuleAngle();
    gyro.getYawPitchRoll(inputs.ypr);
    ypr = inputs.ypr;
  }

  @Override
  public void zeroGyro() {
    gyro.setYaw(-90);
  }

  @Override
  public Rotation2d getYaw() {
    return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
  }

  @Override
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    for (SwerveModule mod : swerveModules) {
      // TODO: Optimize the module state using wpilib optimize method
      // TODO: Check if the optimization is happening in the setDesiredState method
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    Logger.getInstance().recordOutput("SwerveModuleStates", swerveModuleStates);
  }

  @Override
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

    for (SwerveModule mod : getSwerveModules()) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  @Override
  public SwerveModule[] getSwerveModules() {
    return new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
  }
}
