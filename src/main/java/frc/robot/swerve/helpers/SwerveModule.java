// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import static frc.robot.Constants.SwerveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANCoder angleEncoder;
  private double lastAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    CTREConfigs configs = new CTREConfigs();

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder(configs.swerveCanCoderConfig);

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
    configAngleMotor(configs.swerveAngleFXConfig);

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
    configDriveMotor(configs.swerveDriveFXConfig);

    lastAngle = getPosition().angle.getDegrees();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState =
        CTREModuleState.optimize(
            desiredState,
            getPosition().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which CTRE is not

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
      mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
      mDriveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
    // Jittering.
    mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, angleGearRatio));
    lastAngle = angle;
  }

  private void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, angleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder(CANCoderConfiguration config) {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(config);
  }

  private void configAngleMotor(TalonFXConfiguration config) {
    mAngleMotor.configFactoryDefault();
    mAngleMotor.configAllSettings(config);
    mAngleMotor.setInverted(angleMotorInvert);
    mAngleMotor.setNeutralMode(angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor(TalonFXConfiguration config) {
    mDriveMotor.configFactoryDefault();
    mDriveMotor.configAllSettings(config);
    mDriveMotor.setInverted(driveMotorInvert);
    mDriveMotor.setNeutralMode(driveNeutralMode);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition() {
    double velocity = getModuleVelocity();
    Rotation2d angle =
        Rotation2d.fromDegrees(Conversions.falconToDegrees(getModuleAngle(), angleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public double getModuleVelocity() {
    double velocity =
        Conversions.falconToMPS(
            mDriveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio);
    return velocity;
  }

  public double getModuleAngle() {
    return Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearRatio);
  }
}
