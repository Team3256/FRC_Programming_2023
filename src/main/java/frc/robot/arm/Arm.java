// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import static frc.robot.Constants.FeatureFlags.kUsePrefs;
import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.arm.ArmConstants.*;
import static frc.robot.arm.ArmConstants.ArmPreferencesKeys.*;
import static frc.robot.elevator.ElevatorConstants.kElevatorAngleOffset;
import static frc.robot.simulation.SimulationConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.ZeroArm;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.logging.DoubleSendable;
import frc.robot.logging.Loggable;
import frc.robot.swerve.helpers.Conversions;

public class Arm extends SubsystemBase implements CANTestable, Loggable {
  public enum ArmPreset {
    STOW_CUBE(kStowRotationCube),
    STOW_CONE(kStowRotationCone),
    ANY_PIECE_LOW_BACK(kAnyPieceLowBackRotation),
    ANY_PIECE_LOW_FRONT(kAnyPieceLowFrontRotation),
    CUBE_MID(kCubeMidRotation),
    CONE_MID(kConeMidRotation),
    CUBE_HIGH(kCubeHighRotation),
    CONE_HIGH(kConeHighRotation),
    STANDING_CONE_GROUND_INTAKE(kStandingConeGroundIntakeRotation),
    SITTING_CONE_GROUND_INTAKE(kTippedConeGroundIntakeRotation),
    CUBE_GROUND_INTAKE(kCubeGroundIntakeRotation),
    DOUBLE_SUBSTATION_CUBE(kDoubleSubstationRotationCube),
    DOUBLE_SUBSTATION_CONE(kDoubleSubstationRotationCone);

    public final Rotation2d rotation;

    ArmPreset(Rotation2d rotation) {
      this.rotation = rotation;
    }
  }

  private WPI_TalonFX armMotor;
  private final ArmFeedforward armFeedforward = new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);
  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(kArmEncoderDIOPort);

  public Arm() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    System.out.println("Arm initialized");
    zeroArmEncoderElevatorRelative();
    off();

    getLayout(kDriverTabName).add(new ZeroArm(this));
  }

  private void configureRealHardware() {
    armMotor = TalonFXFactory.createDefaultTalon(kArmCANDevice);
    armMotor.setInverted(TalonFXInvertType.CounterClockwise);
    armEncoder.setDistancePerRotation(kArmRadiansPerAbsoluteEncoderRotation);
    armEncoder.reset();

    if (FeatureFlags.kCalibrationMode) {
      armMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      armMotor.setNeutralMode(NeutralMode.Brake);
    }

    if (FeatureFlags.kUseAbsoluteEncoderToInitializeRelative) {
      double absoluteEncoderRadians =
          getAbsoluteEncoderDistanceRad() + kAbsoluteEncoderOffsetRadians;
      armMotor.setSelectedSensorPosition(
          Conversions.radiansToFalcon(absoluteEncoderRadians, kArmGearing));
    } else {
      armMotor.setSelectedSensorPosition(0);
    }
  }

  public double calculateFeedForward(double angleRadians, double velocity) {
    return armFeedforward.calculate(angleRadians, velocity);
  }

  public void setInputVoltage(double voltage) {
    armMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public boolean isSafeFromElevator() {
    return (Units.radiansToDegrees(getArmPositionElevatorRelative()) <= kMaxSafeRotation
        && Units.radiansToDegrees(getArmPositionElevatorRelative()) >= kMinSafeRotation);
  }

  public void zeroArmEncoderElevatorRelative() {
    armMotor.setSelectedSensorPosition(
        Conversions.radiansToFalcon(kElevatorAngleOffset, kArmGearing));
  }

  public double getAbsoluteEncoderDistanceRad() {
    if (FeatureFlags.kUseAbsoluteEncoderToInitializeRelative) {
      return armEncoder.getAbsolutePosition() * kArmRadiansPerAbsoluteEncoderRotation;
    } else {
      return armEncoder.getDistance();
    }
  }

  public Rotation2d getClosestSafePosition(double elevatorPosition) {
    if (elevatorPosition > ElevatorConstants.kSafeForArmMinPosition) {
      return Rotation2d.fromDegrees(
          MathUtil.clamp(
              Units.radiansToDegrees(getArmPositionElevatorRelative()),
              kArmAngleMinConstraint.getDegrees(),
              kArmAngleMaxConstraint.getDegrees()));
    } else {
      return Rotation2d.fromDegrees(
          MathUtil.clamp(
              Units.radiansToDegrees(getArmPositionElevatorRelative()),
              kMinSafeRotation,
              kMaxSafeRotation));
    }
  }

  public double getArmPositionGroundRelative() {
    if (RobotBase.isReal()) {
      if (FeatureFlags.kUseRelativeArmEncoder) {
        return Conversions.falconToRadians(armMotor.getSelectedSensorPosition(), kArmGearing);
      } else {
        double absoluteEncoderDistance;
        if (kUsePrefs) {
          absoluteEncoderDistance =
              getAbsoluteEncoderDistanceRad()
                  + Preferences.getDouble(
                      ArmPreferencesKeys.kAbsoluteEncoderOffsetKey, kAbsoluteEncoderOffsetRadians);
        } else {
          absoluteEncoderDistance = getAbsoluteEncoderDistanceRad() + kAbsoluteEncoderOffsetRadians;
        }
        return absoluteEncoderDistance;
      }
    } else return armSim.getAngleRads();
  }

  public double getArmPositionElevatorRelative() {
    return getArmPositionGroundRelative() - kElevatorAngleOffset;
  }

  public void zeroThroughboreEncoder() {
    armEncoder.reset();
  }

  public boolean isMotorCurrentSpiking() {
    return armMotor.getSupplyCurrent() > kZeroArmCurrentThreshold;
  }

  public void off() {
    armMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    if (Constants.kDebugEnabled) {
      // SmartDashboard.putNumber(
      // "Arm Raw Relative Encoder value", armMotor.getSelectedSensorPosition());
      // SmartDashboard.putNumber("Arm Raw Absolute Encoder value",
      // armEncoder.getAbsolutePosition());
      // SmartDashboard.putNumber(
      // "Arm Raw Absolute Encoder distance", getAbsoluteEncoderDistanceRad());
      SmartDashboard.putNumber("Arm angle ground relative rad", getArmPositionGroundRelative());
      SmartDashboard.putNumber("Arm angle elevator relative rad", getArmPositionElevatorRelative());
      SmartDashboard.putNumber("Arm Current Draw", armMotor.getSupplyCurrent());
      SmartDashboard.putNumber(
          "Arm motor open loop voltage", armMotor.getMotorOutputPercent() * 12);
      SmartDashboard.putBoolean("Arm encoder connected", armEncoder.isConnected());
      SmartDashboard.putNumber(
          "Arm angle position ground relative",
          Units.radiansToDegrees(getArmPositionGroundRelative()));
      SmartDashboard.putNumber(
          "Arm angle position elevator relative",
          Units.radiansToDegrees(getArmPositionElevatorRelative()));
    }
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing arm CAN:");
    boolean result = CANDeviceTester.testTalonFX(armMotor);
    System.out.println("Arm CAN connected: " + result);
    getLayout(kElectricalTabName).add("Arm CAN connected", result);
    return result;
  }

  @Override
  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName)
        .add(
            "Angle",
            new DoubleSendable(() -> Math.toDegrees(getArmPositionGroundRelative()), "Gyro"));
    getLayout(kDriverTabName).add(armMotor);
    getLayout(kDriverTabName).add(new SetArmAngle(this, ArmPreset.STANDING_CONE_GROUND_INTAKE));
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab).getLayout(kArmLayoutName, BuiltInLayouts.kList).withSize(2, 4);
  }

  public Rotation2d getArmSetpoint(Arm.ArmPreset setpoint) {
    if (kUsePrefs) {
      return new Rotation2d(
          Preferences.getDouble(
              ArmPreferencesKeys.kArmPositionKeys.get(setpoint),
              ArmPreferencesKeys.kArmPositionDefaults.get(setpoint).getRadians()));
    } else {
      return setpoint.rotation;
    }
  }

  /** Populating arm preferences on network tables */
  public static void loadArmPreferences() {
    // Arm PID Preferences
    Preferences.initDouble(ArmConstants.ArmPreferencesKeys.kPKey, ArmConstants.kArmP);
    Preferences.initDouble(ArmConstants.ArmPreferencesKeys.kIKey, ArmConstants.kArmI);
    Preferences.initDouble(ArmConstants.ArmPreferencesKeys.kDKey, ArmConstants.kArmD);

    // Arm Encoder Offset
    Preferences.initDouble(
        ArmPreferencesKeys.kAbsoluteEncoderOffsetKey, kAbsoluteEncoderOffsetRadians);
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.STOW_CONE), kStowRotationCone.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.STOW_CUBE), kStowRotationCube.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.ANY_PIECE_LOW_BACK), kAnyPieceLowBackRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.ANY_PIECE_LOW_FRONT),
        kAnyPieceLowFrontRotation.getRadians());
    Preferences.initDouble(kArmPositionKeys.get(ArmPreset.CUBE_MID), kCubeMidRotation.getRadians());
    Preferences.initDouble(kArmPositionKeys.get(ArmPreset.CONE_MID), kConeMidRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.CUBE_HIGH), kCubeHighRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.CONE_HIGH), kConeHighRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.STANDING_CONE_GROUND_INTAKE),
        kStandingConeGroundIntakeRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.SITTING_CONE_GROUND_INTAKE),
        kTippedConeGroundIntakeRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.CUBE_GROUND_INTAKE), kCubeGroundIntakeRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.DOUBLE_SUBSTATION_CONE),
        kDoubleSubstationRotationCone.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(ArmPreset.DOUBLE_SUBSTATION_CUBE),
        kDoubleSubstationRotationCube.getRadians());
  }

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(kNumArmMotors),
          kArmGearing,
          kArmInertia,
          kArmLength,
          kArmAngleMinConstraint.getRadians() + kElevatorAngleOffset,
          kArmAngleMaxConstraint.getRadians() + kElevatorAngleOffset,
          true);

  private MechanismLigament2d armLigament;

  public MechanismLigament2d getLigament() {
    return armLigament;
  }

  private void configureSimHardware() {
    armMotor = new WPI_TalonFX(kArmSimulationID);
    armMotor.setInverted(true);

    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setSelectedSensorPosition(0);

    armLigament =
        new MechanismLigament2d(
            "Arm",
            kArmLength,
            Units.radiansToDegrees(getArmPositionElevatorRelative()) - 90,
            10,
            new Color8Bit(Color.kBlue));
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(armMotor.getMotorOutputPercent() * kVoltage);
    armSim.update(kSimulateDelta);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    simulationOutputToDashboard();
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber(
        "Arm angle position ground relative",
        Units.radiansToDegrees(getArmPositionGroundRelative()));
    SmartDashboard.putNumber(
        "Arm angle position elevator relative",
        Units.radiansToDegrees(getArmPositionElevatorRelative()));
    SmartDashboard.putNumber("Current Draw", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Arm Sim Voltage", armMotor.getMotorOutputPercent() * 12);
  }
}
