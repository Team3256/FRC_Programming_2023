// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import static frc.robot.arm.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.drivers.CanDeviceId;
import frc.robot.drivers.TalonFXFactory;

public class Arm extends SubsystemBase implements CANTestable {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(kArmEncoderDIOPort);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);

  private static final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getFalcon500(kNumArmMotors),
      kArmGearing,
      kArmInertia,
      kArmLengthMeters,
      kArmAngleMinConstraint.getRadians(),
      kArmAngleMaxConstraint.getRadians(),
      kArmMassKg,
      true);

  private final Mechanism2d mechanism2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mechanism2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm = armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow)));

  public Arm() {
    armMotor = TalonFXFactory.createDefaultTalon(new CanDeviceId(kArmMotorID));
    armMotor.setNeutralMode(NeutralMode.Brake);

    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      SmartDashboard.putData("Arm Sim", mechanism2d);
      armTower.setColor(new Color8Bit(Color.kBlue));
    }

    System.out.println("Arm initialized");
    off();
  }

  private void configureRealHardware() {
    armMotor.enableVoltageCompensation(true);
  }

  public double calculateFeedForward(double angleRadians, double velocity) {
    double clampedPosition = MathUtil.clamp(
        angleRadians, kArmAngleMinConstraint.getRadians(), kArmAngleMaxConstraint.getRadians());
    // System.out.println("Position: " + Units.radiansToDegrees(clampedPosition) +
    // ", Velocity: " + velocity);
    SmartDashboard.putNumber("Position setpoint", Units.radiansToDegrees(clampedPosition));
    return armFeedforward.calculate(clampedPosition, velocity);
  }

  public void setInputVoltage(double voltage) {
    armMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public double getArmPositionRads() {
    if (RobotBase.isReal())
      return armEncoder.getAbsolutePosition() * kArmEncoderConversionToRadians;
    else
      return armSim.getAngleRads();
  }

  public void off() {
    armMotor.neutralOutput();
    System.out.println("arm off");
  }

  @Override
  public void periodic() {
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
    SmartDashboard.putNumber("Arm angle position", Units.radiansToDegrees(armSim.getAngleRads()));
    SmartDashboard.putNumber("Current Draw", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Arm Sim Voltage", armMotor.getMotorOutputPercent() * 12);
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing arm CAN:");
    boolean result = CANDeviceTester.testTalonFX(armMotor);
    System.out.println("Arm CAN connected: " + result);
    SmartDashboard.putBoolean("Arm CAN connected", result);
    return result;
  }
}
