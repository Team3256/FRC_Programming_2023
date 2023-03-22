// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole;

import static frc.robot.Constants.ShuffleboardConstants.kDriverTabName;
import static frc.robot.Constants.ShuffleboardConstants.kMoleLayoutName;
import static frc.robot.mole.MoleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.logging.Loggable;
import frc.robot.swerve.helpers.Conversions;

public class Mole extends SubsystemBase implements Loggable, CANTestable {
  public enum MolePreset {
    CUBE_LOW(kDefaultMoleAngle, kDefaultSpeed),
    CUBE_MID(kCubeMidAngle, kCubeMidSpeed),
    CUBE_HIGH(kCubeHighAngle, kCubeHighSpeed);

    public Rotation2d rotation;
    public double desiredSpeed;

    MolePreset(Rotation2d rotation, double desiredSpeed) {
      this.rotation = rotation;
      this.desiredSpeed = desiredSpeed;
    }
  }

  private WPI_TalonFX moleScoreMotor;
  private WPI_TalonFX molePivotMotor;

  private final ArmFeedforward moleFeedforward = new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);

  private static final SingleJointedArmSim moleSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          kMolePivotGearing,
          kMolePivotInertia,
          kMoleLength,
          kMinConstraintAngle.getRadians(),
          kMaxConstraintAngle.getRadians(),
          true);

  private final Mechanism2d mechanism2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d molePivot = mechanism2d.getRoot("MolePivot", 0, 30);
  private final MechanismLigament2d mole =
      molePivot.append(
          new MechanismLigament2d(
              "Mole",
              30,
              Units.radiansToDegrees(moleSim.getAngleRads()),
              6,
              new Color8Bit(Color.kAzure)));

  public Mole() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
    off();
    System.out.println("Mole Intake Initialized");
  }

  private void configureRealHardware() {
    moleScoreMotor = TalonFXFactory.createDefaultTalon(kMoleCANDevice);
    moleScoreMotor.setNeutralMode(NeutralMode.Brake);

    molePivotMotor = TalonFXFactory.createDefaultTalon(kMolePivotCANDevice);
    molePivotMotor.setNeutralMode(NeutralMode.Brake);
  }

  private void configureSimHardware() {
    moleScoreMotor = new WPI_TalonFX(kMoleMotorID);
    moleScoreMotor.setNeutralMode(NeutralMode.Brake);

    molePivotMotor = new WPI_TalonFX(kMolePivotMotorID);
    molePivotMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putData("Mole Sim", mechanism2d);
  }

  public double getMoleSpeed() {
    return moleScoreMotor.getMotorOutputPercent();
  }

  public void intakeCube() {
    System.out.println("Intake Cube");
    moleScoreMotor.set(ControlMode.PercentOutput, kDefaultSpeed);
  }

  public void shootCube(double desiredSpeed) {
    moleScoreMotor.set(ControlMode.PercentOutput, desiredSpeed);
  }

  public double getMolePositionRads() {
    if (RobotBase.isReal())
      return Conversions.falconToRadians(
          molePivotMotor.getSelectedSensorPosition(), kMolePivotGearing);
    else return moleSim.getAngleRads();
  }

  public double calculateFeedForward(double angleRadians, double velocity) {
    return moleFeedforward.calculate(angleRadians, velocity);
  }

  public void setInputVoltage(double voltage) {
    molePivotMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public boolean isCurrentSpiking() {
    return moleScoreMotor.getStatorCurrent() >= kMoleCurrentSpikingThreshold;
  }

  public void off() {
    System.out.println("Mole Off");
    moleScoreMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Mole Stator Current", moleScoreMotor.getStatorCurrent());
  }

  @Override
  public void simulationPeriodic() {
    moleSim.setInput(molePivotMotor.getMotorOutputPercent() * 12);
    moleSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(moleSim.getCurrentDrawAmps()));
    mole.setAngle(Units.radiansToDegrees(moleSim.getAngleRads()));
  }

  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName).add(moleScoreMotor);
  }

  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab).getLayout(kMoleLayoutName, BuiltInLayouts.kList).withSize(2, 4);
  }

  public boolean CANTest() {
    System.out.println("Testing Mole CAN:");
    boolean result = CANDeviceTester.testTalonFX(moleScoreMotor);
    System.out.println("Mole CAN connected" + result);
    SmartDashboard.putBoolean("Mole CAN connected", result);
    return result;
  }
}
