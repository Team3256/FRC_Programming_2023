// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;\
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;

public class CANDeviceTester {
  /**
   * Helper method to test PDP
   *
   * @return Returns whether the PDP is online
   */
  public static boolean testPDP(PowerDistribution device) {
    double voltage = device.getVoltage();
    if (voltage == 0) System.out.println("PDP offline");
    return voltage != 0;
  }

  /**
   * @param device talon fx id to test
   * @return Returns whether all the TalonFXs are online
   */
  public static boolean testTalonFX(WPI_TalonFX device) {
    double temp = device.getTemperature();
    if (temp == 0) System.out.println("TalonFX " + device.getDeviceID() + " offline");
    return temp != 0;
  }

  /**
   * @param device pigeon to test
   * @return Returns whether the Pigeon is online
   */
  public static boolean testPigeon(WPI_PigeonIMU device) {
    double temp = device.getTemp();
    if (temp == 0) System.out.println("Pigeon " + device.getDeviceID() + " offline");
    return temp != 0;
  }

  /**
   * @param device spark max to test
   * @return Returns whether the SparkMax is online
   */
  public static boolean testSparkMax(CANSparkMax device) {
    double temp = device.getMotorTemperature();
    if (temp == 0) System.out.println("SparkMax " + device.getDeviceId() + " offline");
    return temp != 0;
  }

  /**
   * @param device CANCoder to test
   * @return Returns whether the CanCoder is online
   */
  public static boolean testCANCoder(WPI_CANCoder device) {
    double voltage = device.getBusVoltage();
    if (voltage == 0) System.out.println("CANCoder " + device.getDeviceID() + " offline");
    return voltage != 0;
  }
}
