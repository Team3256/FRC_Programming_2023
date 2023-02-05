// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import static frc.robot.arm.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drivers.CanDeviceId;
import frc.robot.drivers.TalonFXFactory;

// TODO: Add
public class ArmSubsystem extends SubsystemBase {
  public class PeriodicIO {
    // Inputs
    public double voltage = 0;
    public double lastUpdateTime = -1;

    // Outputs
    public double angularVelocity = 0;
    public double currentDrawAmps = 0;
  }

  private TalonFX armMotor;
  private SingleJointedArmSim armSim;
  private PeriodicIO periodicIO;

  public ArmSubsystem() {
    periodicIO = new PeriodicIO();
    if (Robot.isReal()) {
      // Configure REAL HW
      armMotor = TalonFXFactory.createDefaultTalon(new CanDeviceId(kArmMotorID));
      armMotor.enableVoltageCompensation(true);
      System.out.println("Arm initalized");
    } else {
      // Configure Sim HW
      armSim =
          new SingleJointedArmSim(
              DCMotor.getFalcon500(1),
              kArmGearing,
              kArmInertia,
              kArmLengthMeters,
              kMinAngleRads,
              kMaxAngleRads,
              kArmMassKg,
              kArmSimGravity);
    }
  }

  public void setInputVoltage(double voltage) {
    periodicIO.voltage = MathUtil.clamp(voltage, 0, 12);
    if (Robot.isSimulation()) {
      armSim.setInputVoltage(periodicIO.voltage);
    } else {
      armMotor.set(TalonFXControlMode.PercentOutput, periodicIO.voltage / 12);
    }
  }

  public double getAngularVelocityRPM() {
    return periodicIO.angularVelocity;
  }

  @Override
  public void simulationPeriodic() {
    double cT = Timer.getFPGATimestamp();
    armSim.update(cT);

    periodicIO.lastUpdateTime = cT;
    periodicIO.angularVelocity = armSim.getVelocityRadPerSec();
    periodicIO.currentDrawAmps = armSim.getCurrentDrawAmps();

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Arm Angular Velocity", periodicIO.angularVelocity);
    SmartDashboard.putNumber("Current Draw", periodicIO.currentDrawAmps);
    SmartDashboard.putNumber("Arm Sim Voltage", periodicIO.voltage);
  }
}
