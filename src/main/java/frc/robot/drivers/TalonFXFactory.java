// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class TalonFXFactory {

  private static final int kTimeoutMs = 100;

  public static class Configuration {
    public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
    // factory default
    public double NEUTRAL_DEADBAND = 0.04;

    public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY =
        SensorInitializationStrategy.BootToZero;
    public double SENSOR_OFFSET_DEGREES = 0;

    public boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
    public boolean ENABLE_STATOR_CURRENT_LIMIT = false;

    public boolean ENABLE_SOFT_LIMIT = false;
    public boolean ENABLE_LIMIT_SWITCH = false;
    public int FORWARD_SOFT_LIMIT = 0;
    public int REVERSE_SOFT_LIMIT = 0;

    public boolean INVERTED = false;
    public boolean SENSOR_PHASE = false;

    public int CONTROL_FRAME_PERIOD_MS = 10;
    public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    // TODO check if this should be 10
    public int GENERAL_STATUS_FRAME_RATE_MS = 20;
    public int FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

    public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
        SensorVelocityMeasPeriod.Period_100Ms;
    public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

    public double OPEN_LOOP_RAMP_RATE = 0.0;
    public double CLOSED_LOOP_RAMP_RATE = 0.0;
  }

  private static final Configuration kDefaultConfiguration = new Configuration();
  private static final Configuration kFollowerConfiguration = new Configuration();

  static {
    // This control frame value seems to need to be something reasonable to avoid
    // the Talon's
    // LEDs behaving erratically. Potentially try to increase as much as possible.
    kFollowerConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
    kFollowerConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    kFollowerConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.ENABLE_SOFT_LIMIT = false;
  }

  // create a CANTalon with the default (out of the box) configuration
  public static WPI_TalonFX createDefaultTalon(CanDeviceId id) {
    return createTalon(id, kDefaultConfiguration);
  }

  public static WPI_TalonFX createPermanentFollowerTalon(
      CanDeviceId followerId, CanDeviceId masterId) {
    if (!followerId.getBus().equals(masterId.getBus())) {
      throw new RuntimeException("Master and Follower Talons must be on the same CAN bus");
    }
    final WPI_TalonFX talon = createTalon(followerId, kFollowerConfiguration);
    talon.set(ControlMode.Follower, masterId.getDeviceNumber());
    return talon;
  }

  public static WPI_TalonFX createTalon(CanDeviceId id, Configuration config) {
    WPI_TalonFX talon = new LazyTalonFX(id);
    talon.set(ControlMode.PercentOutput, 0.0);

    talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
    talon.clearMotionProfileHasUnderrun(kTimeoutMs);
    talon.clearMotionProfileTrajectories();

    talon.clearStickyFaults(kTimeoutMs);

    talon.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, kTimeoutMs);
    talon.configReverseLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, kTimeoutMs);
    talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

    // Turn off re-zeroing by default.
    talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
    talon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);

    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

    talon.configMotorCommutation(MotorCommutation.Trapezoidal);

    talon.configPeakOutputForward(1.0, kTimeoutMs);
    talon.configPeakOutputReverse(-1.0, kTimeoutMs);

    talon.setNeutralMode(config.NEUTRAL_MODE);

    talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
    talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

    talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
    talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
    talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

    talon.setInverted(config.INVERTED);
    talon.setSensorPhase(config.SENSOR_PHASE);

    talon.selectProfileSlot(0, 0);

    talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
    talon.configVelocityMeasurementWindow(
        config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, kTimeoutMs);

    talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
    talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

    talon.configVoltageCompSaturation(0.0, kTimeoutMs);
    talon.configVoltageMeasurementFilter(32, kTimeoutMs);
    talon.enableVoltageCompensation(false);

    talon.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(config.ENABLE_SUPPLY_CURRENT_LIMIT, 20, 60, 0.2),
        kTimeoutMs);
    talon.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(config.ENABLE_STATOR_CURRENT_LIMIT, 20, 60, 0.2),
        kTimeoutMs);

    talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);
    talon.configIntegratedSensorInitializationStrategy(
        config.SENSOR_INITIALIZATION_STRATEGY, kTimeoutMs);
    talon.configIntegratedSensorOffset(config.SENSOR_OFFSET_DEGREES, kTimeoutMs);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_3_Quadrature,
        config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
        kTimeoutMs);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_4_AinTempVbat,
        config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
        kTimeoutMs);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_8_PulseWidth,
        config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
        kTimeoutMs);

    talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

    return talon;
  }
}
