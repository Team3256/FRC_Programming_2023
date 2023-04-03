// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.elevator.ElevatorConstants.*;
import static frc.robot.elevator.ElevatorConstants.ElevatorPreferencesKeys.kElevatorPositionKeys;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    CUBE_HIGH(ElevatorConstants.kCubeHighPositionMeters),
    CONE_HIGH(ElevatorConstants.kConeHighPositionMeters),
    ANY_PIECE_MID(ElevatorConstants.kAnyPieceMidPositionMeters),
    ANY_PIECE_LOW(ElevatorConstants.kAnyPieceLowPositionMeters),
    GROUND_INTAKE(ElevatorConstants.kGroundIntakePositionMeters),
    DOUBLE_SUBSTATION_CONE(ElevatorConstants.kDoubleSubstationPositionConeMeters),
    DOUBLE_SUBSTATION_CUBE(ElevatorConstants.kDoubleSubstationPositionCubeMeters);

    public double position;

    private ElevatorPreset(double position) {
      this.position = position;
    }
  }

  private WPI_TalonFX elevatorMotor;
  private ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(kElevatorS, kElevatorG, kElevatorV, kElevatorA);

  private ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500(kNumElevatorMotors),
          kElevatorGearing,
          kCarriageMass,
          kDrumRadius,
          kMinHeight,
          kMaxHeight,
          true);

  private final Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
      mechanism2dRoot.append(
          new MechanismLigament2d(
              "elevator", Units.metersToInches(elevatorSim.getPositionMeters()), 90));

  public Elevator() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    System.out.println("Elevator initialized");
    off();
  }

  private void configureSimHardware() {
    elevatorMotor = new WPI_TalonFX(kElevatorID);
    SmartDashboard.putData("Elevator Sim", mechanism2d);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  private void configureRealHardware() {
    elevatorMotor = TalonFXFactory.createDefaultTalon(kElevatorCANDevice);
    elevatorMotor.setInverted(kElevatorInverted);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
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
  public void simulationPeriodic() {
    elevatorSim.setInput(elevatorMotor.getMotorOutputPercent() * 12);
    elevatorSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    elevatorMech2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));

    simulationOutputToDashboard();
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

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Elevator position", elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Current Draw", elevatorSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Elevator Sim Voltage", elevatorMotor.getMotorOutputPercent() * 12);
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
    Preferences.initDouble(ElevatorConstants.ElevatorPreferencesKeys.kPKey, ElevatorConstants.kP);
    Preferences.initDouble(ElevatorConstants.ElevatorPreferencesKeys.kIKey, ElevatorConstants.kI);
    Preferences.initDouble(ElevatorConstants.ElevatorPreferencesKeys.kDKey, ElevatorConstants.kD);
    // Elevator Preset Preferences
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.CUBE_HIGH), kCubeHighPositionMeters);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.CONE_HIGH), kConeHighPositionMeters);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.ANY_PIECE_LOW),
        kAnyPieceLowPositionMeters);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.ANY_PIECE_MID),
        kAnyPieceMidPositionMeters);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.GROUND_INTAKE),
        kGroundIntakePositionMeters);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE),
        kDoubleSubstationPositionCubeMeters);
    Preferences.initDouble(
        kElevatorPositionKeys.get(Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE),
        kDoubleSubstationPositionConeMeters);
  }
}
