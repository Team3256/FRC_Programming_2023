// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.arm.ArmConstants.*;

import frc.robot.Robot;
import frc.robot.drivers.CanDeviceId;
import frc.robot.drivers.TalonFXFactory;

// TODO: Add
public class ArmSubsystem extends SubsystemBase {
  public enum ArmPosition {
    HIGH(ArmConstants.kArmHighPositionMeters),
    MID(ArmConstants.kArmMidPositionMeters),
    LOW(ArmConstants.kArmLowPositionMeters);

    public double position;

    private ArmPosition(double position) {
      this.position = position;
    }
  }

  private WPI_TalonFX armMotor;
  private ArmFeedforward armFeedforward =
          new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);

  private SingleJointedArmSim armSim =
          new SingleJointedArmSim(
                  DCMotor.getFalcon500(kNumArmMotors),
                  kArmGearing,
                  jKgMetersSquared,
                  kArmLengthMeters,
                  minAngleRads,
                  maxAngleRads,
                  armMassKg,
                  true);

  private final Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Arm Root", 10, 0);
  private final MechanismLigament2d armMech2d =
          mechanism2dRoot.append(
                  new MechanismLigament2d(
                          "arm", Units.metersToInches(armSim.getAngleRads()), 90));

  public ArmSubsystem() {
    armMotor = new WPI_TalonFX(armID);
    armMotor.setNeutralMode(NeutralMode.Brake);

    if (RobotBase.isReal()) configureRealHardware();
    else SmartDashboard.putData("Arm Sim", mechanism2d);

    System.out.println("Arm initialized");
    off();
  }

  private void configureRealHardware() {
    armMotor = new WPI_TalonFX(armID);
    armMotor.enableVoltageCompensation(true);
  }

  public void zeroArm() {
    armMotor.set(ControlMode.Position, 0);
  }

  public boolean isMotorCurrentSpiking() {
    return armMotor.getSupplyCurrent() >= kArmCurrentThreshold;
  }

  public void off() {
    armMotor.neutralOutput();
    System.out.println("arm off");
  }

  public double calculateFeedForward(double angle, double velocity) {
    return armFeedforward.calculate(angle, velocity);
  }

  public void setInputVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public double getArmPosition() {
    //TODO add falconToMeters
    if (RobotBase.isReal()) return 0;
      else return armSim.getAngleRads();
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(armMotor.getMotorOutputPercent() * 12);
    armSim.update(0.020);

    RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    armMech2d.setLength(Units.metersToInches(armSim.getAngleRads()));

    simulationOutputToDashboard();
  }

  @Override
  public void periodic() {}

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Arm position", armSim.getAngleRads());
    SmartDashboard.putNumber("Current Draw", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Arm Sim Voltage", armMotor.getMotorOutputPercent() * 12);
  }
}
