// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.arm.ArmConstants.*;
import static frc.robot.arm.ArmConstants.ArmPreferencesKeys.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.logging.DoubleSendable;
import frc.robot.logging.Loggable;
import frc.robot.swerve.helpers.Conversions;

public class Arm extends SubsystemBase implements CANTestable, Loggable {
  public enum ArmPreset {
    DEFAULT(ArmConstants.kDefaultArmAngle),
    ANY_PIECE_LOW(ArmConstants.kAnyPieceLowRotation),
    CUBE_MID(ArmConstants.kCubeMidRotation),
    CONE_MID(ArmConstants.kConeMidRotation),
    CUBE_HIGH(ArmConstants.kCubeHighRotation),
    CONE_HIGH(ArmConstants.kConeHighRotation),
    GROUND_INTAKE(ArmConstants.kGroundIntakeRotation),
    DOUBLE_SUBSTATION_CUBE(ArmConstants.kDoubleSubstationRotationCube),
    DOUBLE_SUBSTATION_CONE(ArmConstants.kDoubleSubstationRotationCone);

    public Rotation2d rotation;

    private ArmPreset(Rotation2d rotation) {
      this.rotation = rotation;
    }
  }

  private WPI_TalonFX armMotor;
  private final ArmFeedforward armFeedforward = new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);
  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(kArmEncoderDIOPort);

  private static final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(kNumArmMotors),
          kArmGearing,
          kArmInertia,
          kArmLengthMeters,
          kArmAngleMinConstraint.getRadians(),
          kArmAngleMaxConstraint.getRadians(),
          true);

  private final Mechanism2d mechanism2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mechanism2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public Arm() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    System.out.println("Arm initialized");
    off();
  }

  private void configureSimHardware() {
    armMotor = new WPI_TalonFX(kArmSimulationID);
    SmartDashboard.putData("Arm Sim", mechanism2d);
    armTower.setColor(new Color8Bit(Color.kBlue));
  }

  private void configureRealHardware() {
    armMotor = TalonFXFactory.createDefaultTalon(kArmCANDevice);
    armMotor.setInverted(true);
    armEncoder.setDistancePerRotation(kArmRadiansPerAbsoluteEncoderRotation);

    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setSelectedSensorPosition(0);
  }

  public void setCoast() {
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrake() {
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double calculateFeedForward(double angleRadians, double velocity) {
    return armFeedforward.calculate(angleRadians, velocity);
  }

  public void setInputVoltage(double voltage) {
    armMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public void zeroEncoder() {
    armMotor.setSelectedSensorPosition(0);
  }

  /**
   * Reset encoder offset. Use when you know where the arm actually is in space but the relative
   * encoder is off. Useful when the gear skips and you need to change the offset
   *
   * @param currentAbsolutePosition The actual position of the arm in space that you want the
   *     current relative encoder value to reflect. This will change all setpoint for the arm.
   */
  public void resetOffset(Rotation2d currentAbsolutePosition) {
    ArmConstants.kRelativeFalconEncoderOffsetRadians =
        ArmConstants.kRelativeFalconEncoderOffsetRadians
            + (currentAbsolutePosition.getRadians() - this.getArmPositionRads());

    System.out.println("New arm offset" + ArmConstants.kRelativeFalconEncoderOffsetRadians);
  }

  public double getArmPositionRads() {
    if (RobotBase.isReal()) {
      if (Constants.FeatureFlags.kArmAbsoluteEncoderEnabled) {
        double absoluteEncoderDistance =
            armEncoder.getDistance()
                + Preferences.getDouble(
                    ArmPreferencesKeys.kAbsoluteEncoderOffsetKey, kAbsoluteEncoderOffsetRadians);
        if (absoluteEncoderDistance < kArmAngleMinConstraint.getRadians()) {
          return absoluteEncoderDistance + Math.PI * 2;
        } else {
          return absoluteEncoderDistance;
        }
      } else
        return Conversions.falconToRadians(armMotor.getSelectedSensorPosition(), kArmGearing)
            + Preferences.getDouble(
                ArmPreferencesKeys.kRelativeEncoderOffsetKey, kRelativeFalconEncoderOffsetRadians);
    } else return armSim.getAngleRads();
  }

  public void off() {
    armMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    if (Constants.kDebugEnabled) {
      SmartDashboard.putNumber(
          "Arm Raw Relative Encoder value", armMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Raw Absolute Encoder value", armEncoder.getDistance());
      SmartDashboard.putNumber("Arm angle", Units.radiansToDegrees(getArmPositionRads()));
      SmartDashboard.putNumber("Current Draw", armSim.getCurrentDrawAmps());
      SmartDashboard.putNumber(
          "Arm motor open loop voltage", armMotor.getMotorOutputPercent() * 12);
      SmartDashboard.putBoolean("Arm encoder connected", armEncoder.isConnected());
    }
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(armMotor.getMotorOutputPercent() * 12);
    armSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    simulationOutputToDashboard();
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Arm angle position", Units.radiansToDegrees(getArmPositionRads()));
    SmartDashboard.putNumber("Current Draw", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Arm Sim Voltage", armMotor.getMotorOutputPercent() * 12);
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
        .add("Angle", new DoubleSendable(() -> Math.toDegrees(getArmPositionRads()), "Gyro"));
    getLayout(kDriverTabName).add(armMotor);
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab).getLayout(kArmLayoutName, BuiltInLayouts.kList).withSize(2, 4);
  }

  public Rotation2d getArmSetpoint(Arm.ArmPreset setpoint) {
    if (Constants.FeatureFlags.kUsePrefs) {
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
    Preferences.initDouble(ArmConstants.ArmPreferencesKeys.kPKey, ArmConstants.kP);
    Preferences.initDouble(ArmConstants.ArmPreferencesKeys.kIKey, ArmConstants.kI);
    Preferences.initDouble(ArmConstants.ArmPreferencesKeys.kDKey, ArmConstants.kD);

    // Arm Encoder Offset
    Preferences.initDouble(
        ArmPreferencesKeys.kRelativeEncoderOffsetKey, kRelativeFalconEncoderOffsetRadians);
    Preferences.initDouble(
        ArmPreferencesKeys.kAbsoluteEncoderOffsetKey, kAbsoluteEncoderOffsetRadians);
    // Arm Preset Preferences
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.DEFAULT), kDefaultArmAngle.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.ANY_PIECE_LOW), kAnyPieceLowRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.CUBE_MID), kCubeMidRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.CONE_MID), kConeMidRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.CUBE_HIGH), kCubeHighRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.CONE_HIGH), kConeHighRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.GROUND_INTAKE), kGroundIntakeRotation.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.DOUBLE_SUBSTATION_CONE),
        kDoubleSubstationRotationCone.getRadians());
    Preferences.initDouble(
        kArmPositionKeys.get(Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE),
        kDoubleSubstationRotationCube.getRadians());
  }
}
