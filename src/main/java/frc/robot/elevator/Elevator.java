// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.elevator.ElevatorConstants.*;
import static frc.robot.elevator.ElevatorConstants.ElevatorPreferencesKeys.*;
import static frc.robot.simulation.SimulationConstants.*;
import static frc.robot.swerve.helpers.Conversions.falconToMeters;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeatureFlags;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.logging.DoubleSendable;
import frc.robot.logging.Loggable;

public class Elevator extends SubsystemBase implements CANTestable, Loggable {
  public enum ElevatorPreset {
    STOW_CONE(kConeStowPosition),
    STOW_CUBE(kCubeStowPosition),
    CUBE_HIGH(kCubeHighPosition),
    CONE_HIGH(kConeHighPosition),
    ANY_PIECE_MID(kAnyPieceMidPosition),
    ANY_PIECE_LOW(kAnyPieceLowPosition),
    GROUND_INTAKE(kGroundIntakePosition),
    DOUBLE_SUBSTATION_CONE(kConeDoubleSubstationPosition),
    DOUBLE_SUBSTATION_CUBE(kCubeDoubleSubstationPosition);

    public final double position;

    ElevatorPreset(double position) {
      this.position = position;
    }
  }

  private WPI_TalonFX elevatorMotor;
  private WPI_TalonFX elevatorFollowerMotor;
  private ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(kElevatorS, kElevatorG, kElevatorV, kElevatorA);

  public Elevator() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    System.out.println("Elevator initialized");
    off();
  }

  private void configureRealHardware() {
    elevatorMotor = TalonFXFactory.createDefaultTalon(kElevatorCANDevice);
    elevatorMotor.setInverted(true);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorFollowerMotor =
        TalonFXFactory.createPermanentFollowerTalon(kElevatorFollowerCANDevice, kElevatorCANDevice);
    elevatorFollowerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public boolean isMotorCurrentSpiking() {
    if (RobotBase.isReal()) {
      return elevatorMotor.getSupplyCurrent() >= kElevatorCurrentThreshold;
    } else {
      return elevatorSim.getCurrentDrawAmps() >= kElevatorCurrentThreshold;
    }
  }

  public double calculateFeedForward(double velocity) {
    return elevatorFeedforward.calculate(velocity);
  }

  public void setInputVoltage(double voltage) {
    SmartDashboard.putNumber("Elevator Voltage", voltage);
    elevatorMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public double getElevatorPosition() {
    if (RobotBase.isReal()) {
      return falconToMeters(
          elevatorMotor.getSelectedSensorPosition(), 2 * Math.PI * kDrumRadius, kElevatorGearing);
    } else return elevatorSim.getPositionMeters();
  }

  public void setCoast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void zeroElevator() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public void off() {
    elevatorMotor.neutralOutput();
    System.out.println("Elevator off");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Elevator position inches", Units.metersToInches(getElevatorPosition()));
    SmartDashboard.putNumber("Elevator Current Draw", elevatorMotor.getSupplyCurrent());
  }

  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName).add(new ZeroElevator(this));
    getLayout(kDriverTabName).add("Position", new DoubleSendable(this::getElevatorPosition));
    getLayout(kDriverTabName).add(elevatorMotor);
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab)
        .getLayout(kElevatorLayoutName, BuiltInLayouts.kList)
        .withSize(2, 4);
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing Elevator CAN:");
    boolean result = CANDeviceTester.testTalonFX(elevatorMotor);
    System.out.println("Elevator CAN connected: " + result);
    getLayout(kElectricalTabName).add("Elevator CAN connected", result);
    return result;
  }

  public double getElevatorSetpoint(Elevator.ElevatorPreset setpoint) {
    if (FeatureFlags.kUsePrefs) {
      return Preferences.getDouble(
          ElevatorPreferencesKeys.kElevatorPositionKeys.get(setpoint),
          ElevatorPreferencesKeys.kElevatorPositionDefaults.get(setpoint));
    } else {
      return setpoint.position;
    }
  }

  /** Populating elevator preferences on network tables */
  public static void loadElevatorPreferences() {
    // Elevator PID Preferences
    Preferences.initDouble(ElevatorPreferencesKeys.kPKey, kElevatorP);
    Preferences.initDouble(ElevatorPreferencesKeys.kIKey, kElevatorI);
    Preferences.initDouble(ElevatorPreferencesKeys.kDKey, kElevatorD);
    // Elevator Preset Preferences
    Preferences.initDouble(kElevatorPositionKeys.get(ElevatorPreset.STOW_CONE), kConeStowPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.STOW_CUBE), kCubeStowPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.CUBE_HIGH), kCubeHighPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.CONE_HIGH), kConeHighPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.ANY_PIECE_LOW), kAnyPieceLowPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.ANY_PIECE_MID), kAnyPieceMidPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.GROUND_INTAKE), kGroundIntakePosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE),
        kCubeDoubleSubstationPosition);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE),
        kConeDoubleSubstationPosition);
  }

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500(kNumElevatorMotors),
          kElevatorGearing,
          kCarriageMass,
          kDrumRadius,
          kMinExtension,
          kMaxExtension,
          true);
  private MechanismLigament2d elevatorLigament;

  private void configureSimHardware() {
    elevatorMotor = new WPI_TalonFX(kElevatorMasterID);
    elevatorMotor.setInverted(true);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorLigament =
        new MechanismLigament2d(
            "Elevator",
            Units.inchesToMeters(kArmStartPosition) + elevatorSim.getPositionMeters(),
            Units.radiansToDegrees(kElevatorAngleOffset),
            kElevatorLineWidth,
            new Color8Bit(Color.kRed));
  }

  public MechanismLigament2d getLigament() {
    return elevatorLigament;
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(elevatorMotor.getMotorOutputPercent() * kVoltage);
    elevatorSim.update(kSimulateDelta);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    simulationOutputToDashboard();
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Elevator position", elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Current Draw", elevatorSim.getCurrentDrawAmps());
    SmartDashboard.putNumber(
        "Elevator Sim Voltage", elevatorMotor.getMotorOutputPercent() * kVoltage);
  }
}
