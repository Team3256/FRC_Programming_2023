// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.climb;

import static frc.robot.climb.ClimbConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;

public class Climb extends SubsystemBase implements CANTestable {
  private WPI_TalonFX climbMotor;

  public Climb() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    off();
    System.out.println("Climb initialized");
  }

  public void configureRealHardware() {
    climbMotor = TalonFXFactory.createDefaultTalon(kClimbCANDevice);
    climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void configureSimHardware() {
    climbMotor = new WPI_TalonFX(kClimbMotorID);
    climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void deployClimb() {
    double deploySpeed =
        Preferences.getDouble(ClimbPreferencesKeys.kClimbDeploySpeedKey, kClimbDeploySpeed);
    climbMotor.set(ControlMode.PercentOutput, deploySpeed);
  }

  public void retractClimb() {
    double retractSpeed =
        Preferences.getDouble(ClimbPreferencesKeys.kClimbRetractSpeedKey, kClimbRetractSpeed);
    climbMotor.set(ControlMode.PercentOutput, retractSpeed);
  }

  public void off() {
    System.out.println("Climb Off");
    climbMotor.neutralOutput();
  }

  public static void loadClimbPreferences() {
    Preferences.initDouble(
        ClimbConstants.ClimbPreferencesKeys.kClimbDeployRotationKey,
        ClimbConstants.kClimbDeployRotation);
    Preferences.initDouble(
        ClimbConstants.ClimbPreferencesKeys.kClimbRetractRotationKey,
        ClimbConstants.kClimbRetractRotation);
  }

  public boolean CANTest() {
    System.out.println("Testing Climb CAN:");
    boolean result = CANDeviceTester.testTalonFX(climbMotor);
    System.out.println("Climb CAN connected: " + result);
    SmartDashboard.putBoolean("Climb CAN connected", result);
    return result;
  }
}
