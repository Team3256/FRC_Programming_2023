// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import static frc.robot.swerve.SwerveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivers.CANDeviceTester;

public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private WPI_TalonFX mAngleMotor;
  private WPI_TalonFX mDriveMotor;
  private WPI_CANCoder angleEncoder;
  private double lastAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kDriveKS, kDriveKV, kDriveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    CTREConfigs configs = new CTREConfigs();

    /* Angle Encoder Config */
    angleEncoder = new WPI_CANCoder(moduleConstants.canCoderID);
    configAngleEncoder(configs.swerveCanCoderConfig);

    /* Angle Motor Config */
    mAngleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
    configAngleMotor(configs.swerveAngleFXConfig);

    /* Drive Motor Config */
    mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
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
      double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
      mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond, kWheelCircumference, kDriveGearRatio);
      mDriveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
    // Jittering.
    mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, kAngleGearRatio));
    lastAngle = angle;
  }

  public void setDesiredAngleState(SwerveModuleState desiredState) {
    desiredState = CTREModuleState.optimize(desiredState, getPosition().angle);

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle.getDegrees();

    mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, kAngleGearRatio));
    lastAngle = angle;
  }

  private void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, kAngleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder(CANCoderConfiguration config) {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(config);
  }

  private void configAngleMotor(TalonFXConfiguration config) {
    mAngleMotor.configFactoryDefault();
    mAngleMotor.configAllSettings(config);
    mAngleMotor.setInverted(kAngleMotorInvert);
    mAngleMotor.setNeutralMode(kAngleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor(TalonFXConfiguration config) {
    mDriveMotor.configFactoryDefault();
    mDriveMotor.configAllSettings(config);
    mDriveMotor.setInverted(kDriveMotorInvert);
    mDriveMotor.setNeutralMode(kDriveNeutralMode);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  public void setDriveMotorNeutralMode(NeutralMode neutralMode) {
    mDriveMotor.setNeutralMode(neutralMode);
  }

  public void setAngleMotorNeutralMode(NeutralMode neutralMode) {
    mAngleMotor.setNeutralMode(neutralMode);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition() {
    double velocity =
        Conversions.falconToMPS(
            mDriveMotor.getSelectedSensorPosition(), kWheelCircumference, kDriveGearRatio);
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), kAngleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public WPI_TalonFX getAngleMotor() {
    return mAngleMotor;
  }

  public WPI_TalonFX getDriveMotor() {
    return mDriveMotor;
  }

  public WPI_CANCoder getAngleEncoder() {
    return angleEncoder;
  }

  public boolean test() {
    System.out.println("Testing swerve module CAN:");
    boolean result =
        CANDeviceTester.testTalonFX(mDriveMotor)
            && CANDeviceTester.testTalonFX(mAngleMotor)
            && CANDeviceTester.testCANCoder(angleEncoder);
    System.out.println("Swerve module CAN connected: " + result);
    SmartDashboard.putBoolean("Swerve module CAN connected", result);
    return result;
  }
}
