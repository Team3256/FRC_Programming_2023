// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import static frc.robot.arm.ArmConstants.*;
import static frc.robot.swerve.helpers.Conversions.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import frc.robot.drivers.CanDeviceId;
import frc.robot.drivers.TalonFXFactory;

public class Arm extends SubsystemBase {

  private static WPI_TalonFX armMotor;
  private final ArmFeedforward armFeedforward = new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);

  private static final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(kNumArmMotors),
          kArmGearing,
          jKgMetersSquared,
          kArmLengthMeters,
          kArmAngleConstraint,
          (180 - kArmAngleConstraint),
          armMassKg,
          true);

  private final Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Arm Root", 10, 0);
  private final MechanismLigament2d armMech2d =
      mechanism2dRoot.append(
          new MechanismLigament2d(
              "arm", kArmLengthMeters, Units.radiansToDegrees(armSim.getAngleRads())));

  public Arm() {
    armMotor = TalonFXFactory.createDefaultTalon(new CanDeviceId(kArmMotorID));
    armMotor.setNeutralMode(NeutralMode.Brake);

    if (RobotBase.isReal()) configureRealHardware();
    else SmartDashboard.putData("Arm Sim", mechanism2d);

    System.out.println("Arm initialized");
    off();
  }

  private void configureRealHardware() {
    armMotor.enableVoltageCompensation(true);
  }

  public double calculateFeedForward(double angle, double velocity) {
    return armFeedforward.calculate(angle, velocity);
  }

  public void setInputVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public double getArmPosition() {
    if (RobotBase.isReal()) {
      return falconToRadians(armMotor.getSelectedSensorPosition(), kArmGearing);
    } else return armSim.getAngleRads();
  }

  public void off() {
    armMotor.neutralOutput();
    System.out.println("arm off");
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
    SmartDashboard.putNumber("Arm angle position", armSim.getAngleRads());
    SmartDashboard.putNumber("Current Draw", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Arm Sim Voltage", armMotor.getMotorOutputPercent() * 12);
  }
}