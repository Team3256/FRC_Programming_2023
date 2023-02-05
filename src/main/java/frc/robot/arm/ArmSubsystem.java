// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

// TODO: Add
public class ArmSubsystem extends SubsystemBase {
  public class PeriodicIO {
    // Inputs
    public double voltage = 0;
    public double lastUpdateTime = -1;

    // Outputs
    public double angleRad = 0;
    public double currentDrawAmps = 0;
    public double angularVelocityRadPerSec = 0;
    public double angularAccelerationRadPerSecSq = 0;
  }

  private TalonFX armMotor;
  private SingleJointedArmSim armSim;
  private PeriodicIO periodicIO;

  private ArmFeedforward armFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  public ArmSubsystem() {
    periodicIO = new PeriodicIO();
    if (Robot.isReal()) {
      // Configure REAL HW
      armMotor = new TalonFX(ARM_MOTOR_ID);
      armMotor.enableVoltageCompensation(true);
      System.out.println("Arm initalized");
    } else {
      // Configure Sim HW
      System.out.println("Arm initalized in simulation.");
      armSim =
          new SingleJointedArmSim(
              DCMotor.getFalcon500(1),
              Constants.ArmConstants.kArmGearing,
              Constants.ArmConstants.kArmInertia,
              Constants.ArmConstants.kArmLengthMeters,
              Constants.ArmConstants.kMinAngleRads,
              Constants.ArmConstants.kMaxAngleRads,
              Constants.ArmConstants.kArmMassKg,
              Constants.ArmConstants.kArmSimGravity);
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

  public double getAngle() {
    return periodicIO.angleRad;
  }

  public double calculateFFVelocity(double desiredAngle, double desiredVelocity) {
    return armFeedforward.calculate(desiredAngle, desiredVelocity);
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    armSim.update(currentTime);

    periodicIO.angleRad = armSim.getAngleRads();
    periodicIO.currentDrawAmps = armSim.getCurrentDrawAmps();
    periodicIO.angularAccelerationRadPerSecSq =
        (armSim.getVelocityRadPerSec() - periodicIO.angularVelocityRadPerSec)
            / (currentTime - periodicIO.lastUpdateTime);
    periodicIO.angularVelocityRadPerSec = armSim.getVelocityRadPerSec();
    periodicIO.lastUpdateTime = currentTime;

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Arm Angle", periodicIO.angleRad);
    SmartDashboard.putNumber("Current Draw", periodicIO.currentDrawAmps);
    SmartDashboard.putNumber("Arm Sim Voltage", periodicIO.voltage);
    SmartDashboard.putNumber("Arm Velocity", periodicIO.angularVelocityRadPerSec);
    SmartDashboard.putNumber("Arm Acceleration", periodicIO.angularAccelerationRadPerSecSq);
  }
}
