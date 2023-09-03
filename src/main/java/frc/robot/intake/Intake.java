// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import static frc.robot.Constants.ShuffleboardConstants.kDriverTabName;
import static frc.robot.Constants.ShuffleboardConstants.kIntakeLayoutName;
import static frc.robot.Constants.kDebugEnabled;
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.simulation.SimulationConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.logging.Loggable;

public class Intake extends SubsystemBase implements Loggable, CANTestable {
  private WPI_TalonFX intakeMotor;
  private TimeOfFlight leftDistanceSensor;
  private TimeOfFlight rightDistanceSensor;

  private double leftDistance;
  private double rightDistance;

  public Intake() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
    off();
    System.out.println("Intake initialized");
  }

  private void configureRealHardware() {
    intakeMotor = TalonFXFactory.createDefaultTalon(kIntakeCANDevice);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    configIntakeCurrentLimit();

    if (FeatureFlags.kIntakeAutoScoreDistanceSensorOffset) {
      leftDistanceSensor = new TimeOfFlight(kLeftDistanceSensorID);
      rightDistanceSensor = new TimeOfFlight(kRightDistanceSensorID);

      leftDistanceSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.05);
      rightDistanceSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.05);

      if (Constants.kDebugEnabled) {
        SmartDashboard.putData("Intake motor", intakeMotor);
        SmartDashboard.putData("Left distance sensor", leftDistanceSensor);
        SmartDashboard.putData("Right distance sensor", rightDistanceSensor);
      }
    }
  }

  public void configIntakeCurrentLimit() {
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 60, 0.2));
  }

  public double getGamePieceOffset() {
    if (FeatureFlags.kIntakeAutoScoreDistanceSensorOffset) {
      updateSensorDistances();
      return (rightDistance - leftDistance) / 2;
    }
    return 0;
  }

  public void updateSensorDistances() {
    double leftMeasurement = leftDistanceSensor.getRange() / 1000;
    double rightMeasurement = rightDistanceSensor.getRange() / 1000;
    if (leftDistanceSensor.isRangeValid()) leftDistance = leftMeasurement;
    if (rightDistanceSensor.isRangeValid()) rightDistance = rightMeasurement;
  }

  public double getIntakeSpeed() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void latchCone() {
    intakeMotor.set(ControlMode.PercentOutput, kLatchConeSpeed);
  }

  public void latchCube() {
    intakeMotor.set(ControlMode.PercentOutput, kLatchCubeSpeed);
  }

  public void configureLatchCurrentLimit(boolean enabled) {
    if (kDebugEnabled) System.out.println("Setting Current Limit Configuration: " + enabled);
    intakeMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            enabled, kGamePieceMaxCurrent, kIntakeMaxCurrent, kTriggerThresholdTime));
  }

  public void intakeCone() {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeConeSpeed);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeConeSpeed);
  }

  public void outtakeCone() {
    intakeMotor.set(ControlMode.PercentOutput, kOuttakeConeSpeed);
  }

  public void intakeCube() {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeCubeSpeed);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeCubeSpeed);
  }

  public void outtakeCube() {
    intakeMotor.set(ControlMode.PercentOutput, kOuttakeCubeSpeed);
  }

  public boolean isCurrentSpiking() {
    return intakeMotor.getSupplyCurrent() > kIntakeMaxCurrent;
  }

  public void off() {
    intakeMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    if (Constants.kDebugEnabled) {
      SmartDashboard.putNumber("Intake supply current", intakeMotor.getSupplyCurrent());
      SmartDashboard.putNumber("Intake stator current", intakeMotor.getStatorCurrent());
      SmartDashboard.putNumber("Intake game piece offset from center", getGamePieceOffset());
    }
  }

  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName).add(new IntakeCube(this));
    getLayout(kDriverTabName).add(new IntakeCone(this));
    getLayout(kDriverTabName).add(intakeMotor);
  }

  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab)
        .getLayout(kIntakeLayoutName, BuiltInLayouts.kList)
        .withSize(2, 4);
  }

  public boolean CANTest() {
    System.out.println("Testing intake CAN:");
    boolean result = CANDeviceTester.testTalonFX(intakeMotor);
    System.out.println("Intake CAN connected: " + result);
    SmartDashboard.putBoolean("Intake CAN connected", result);
    return result;
  }

  private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 1);
  private MechanismLigament2d intakePivot;

  public MechanismLigament2d getWrist() {
    return intakePivot;
  }

  private void configureSimHardware() {
    intakeMotor = new WPI_TalonFX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakePivot =
        new MechanismLigament2d(
            "Intake Wrist", Units.inchesToMeters(2.059), -90, 0, new Color8Bit(Color.kBlack));
  }

  @Override
  public void simulationPeriodic() {
    intakeSim.setInput(intakeMotor.getMotorOutputPercent() * kVoltage);
    intakeSim.update(kSimulateDelta);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));
    simulationOutputToDashboard();
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber(
        "Intake angle deg", Units.radiansToDegrees(intakeSim.getAngularPositionRad()));
    SmartDashboard.putNumber("Intake current draw", intakeMotor.getStatorCurrent());
    SmartDashboard.putNumber("Intake sim voltage", intakeMotor.getMotorOutputPercent());
  }
}
