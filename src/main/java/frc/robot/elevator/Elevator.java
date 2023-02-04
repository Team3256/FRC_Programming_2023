// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import static frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.swerve.helpers.Conversions.falconToDistance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMotor;
  private ElevatorSim elevatorSim;
  private double lastUpdateTime = 0;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.1, 3, 1);

  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorID);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
    System.out.println("Elevator initialized");
    off();
  }

  public double calculateFeedForward(double setpoint) {
    return feedforward.calculate(setpoint * maxVelocity);
  }

  public void zeroElevator() {
    elevatorMotor.set(ControlMode.Position, 0);
  }

  public boolean isMotorCurrentSpiking() {
    return elevatorMotor.getSupplyCurrent() >= kElevatorCurrentThreshold;
  }

  public void off() {
    if (RobotBase.isReal()) elevatorMotor.neutralOutput();
    else elevatorSim.setInputVoltage(0);
    System.out.println("Elevator off");
  }

  private void configureRealHardware() {
    elevatorMotor = new TalonFX(elevatorID);
    elevatorMotor.enableVoltageCompensation(true);
  }

  private void configureSimHardware() {
    elevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            elevatorGearRatio,
            kCarriageMass,
            elevatorDrumRadius,
            kMinElevatorHeight,
            kMaxElevatorHeight,
            true);
  }

  public void stopElevator() {
    elevatorMotor.neutralOutput();
  }

  public void setPercentSpeed(double percentSpeed) {
    elevatorMotor.set(ControlMode.PercentOutput, percentSpeed);
    if (RobotBase.isSimulation()) elevatorSim.setInputVoltage(percentSpeed * 12);
  }

  public double getElevatorPosition() {
    if (RobotBase.isReal()) {
      return falconToDistance(
          elevatorMotor.getSelectedSensorPosition(), elevatorMotorDiameter, elevatorGearRatio);
    } else return elevatorSim.getPositionMeters();
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    if (lastUpdateTime != 0) elevatorSim.update(currentTime - lastUpdateTime);
    else elevatorSim.update(0.2);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    simulationOutputToDashboard();
    lastUpdateTime = currentTime;
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Elevator position", elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Current Draw", elevatorSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Elevator Sim Voltage", elevatorMotor.getMotorOutputPercent() * 12);
  }
}
