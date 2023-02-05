// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.arm.commands.SetArmFromPID;
import frc.robot.arm.commands.SetArmFromVoltage;
import org.junit.*;
import org.junit.Assert.*;

public class ArmUnitTest {

  static ArmSubsystem armSubsystem;

  @Before
  public static void setup() {
    assert HAL.initialize(500, 0); // Initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    armSubsystem = new ArmSubsystem();
  }

  // SetArmFromVoltage test (one routine)
  @Test
  public void setVoltageRoutine5() {
    SetArmFromVoltage setVoltageTo5Volts = new SetArmFromVoltage(armSubsystem, 5.0);
    setVoltageTo5Volts.schedule();
    runScheduler(1);
    assertEquals("Set voltage to 5", ArmSubsystem.getVoltage(), 5.0, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageTo0Volts = new SetArmFromVoltage(armSubsystem, 0.0);
    setVoltageTo0Volts.schedule();
    runScheduler(1);
    assertEquals("Set voltage to 0", ArmSubsystem.getVoltage(), 0.0, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageBackTo5Volts = new SetArmFromVoltage(armSubsystem, 5.0);
    setVoltageBackTo5Volts.schedule();
    runScheduler(1);
    assertEquals("Set voltage to 5", ArmSubsystem.getVoltage(), 5.0, DELTA);
    wait(1000);
  }

  // SetArmFromPID test (one routine)

  @Test
  public void setArmFromPIDRoutine() {
    SetArmFromPID setArmPIDTo600 = new SetArmFromPID(armSubsystem, 600);
    setArmPIDTo600.schedule();
    runScheduler(1.5);
    assertEquals(
        "Set angular velocity to 600 RPM", armSubsystem.getAngularVelocityRPM(), 600, DELTA);
    wait(1000);

    SetArmFromPID setArmPIDTo0 = new SetArmFromPID(armSubsystem, 0);
    setArmPIDTo0.schedule();
    runScheduler(1);
    assertEquals("Set angular velocity to 0 RPM", armSubsystem.getAngularVelocityRPM(), 0, DELTA);
    wait(1000);

    SetArmFromPID setArmPIDTo1200 = new SetArmFromPID(armSubsystem, 1200);
    setArmPIDTo1200.schedule();
    runScheduler(1.5);
    assertEquals(
        "Set angular velocity to 1200 RPM", armSubsystem.getAngularVelocityRPM(), 1200, DELTA);
    setArmPIDTo1200.end();
  }

  // Run scheduler

  private static void runScheduler(double seconds) {
    try {
      for (int i = 0; i < seconds * 1000 / 20; ++i) {
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        // TODO: Change to motor the arm is using
        CommandScheduler.getInstance().run();
        Thread.sleep(20);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
