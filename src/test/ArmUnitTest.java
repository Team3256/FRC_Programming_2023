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

  // Subsystem tests
  @Test
  public void setInputVoltageTo0() {
    ArmSubsystem setInputVoltageTo0Command = new ArmSubsystem(0.0);
    DriverStationSim.setEnabled(true);
    assertEquals("Set input voltage to 0", ArmSubsystem.getVoltage(), 0.0, DELTA);
  }

  // SetArmFromVoltage tests
  @Test
  public void setVoltageTo5Volts() {
    SetArmFromVoltage setVoltageTo5Command = new SetArmFromVoltage(armSubsystem, 5.5);
    setVoltageTo5Command.schedule();
    runScheduler();
    DriverStationSim.setEnabled(true);
    assertEquals("Set voltage to 0", ArmSubsystem.getVoltage(), 5.0, DELTA);
  }

  @Test
  public void setVoltageRoutine() {
    SetArmFromVoltage setVoltageTo5Volts = new SetArmFromVoltage(armSubsystem, 0.5);
    setVoltageTo5Volts.schedule();
    runScheduler();
    DriverStationSim.setEnabled(true);
    assertEquals("Set voltage to 5", ArmSubsystem.getVoltage(), 5.0, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageTo0Volts = new SetArmFromVoltage(armSubsystem, 0.0);
    setVoltageTo0Volts.schedule();
    runScheduler();
    assertEquals("Set voltage to 0", ArmSubsystem.getVoltage(), 0.0, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageTo5Volts = new SetArmFromVoltage(armSubsystem, 0.5);
    setVoltageTo5Volts.schedule();
    runScheduler();
    assertEquals("Set voltage to 5", ArmSubsystem.getVoltage(), 5.0, DELTA);
    wait(1000);
  }

  // SetArmFromPID
  @Test
  public void setArmFromPID() {
    SetArmFromPID setArmFromPIDCommand = new SetArmFromPID(armSubsystem, 1200);
    setArmFromPIDCommand.schedule();
    runScheduler();
    DriverStationSim.setEnabled(true);
    assertEquals(
        "Set angular velocity to 1200 RPM", ArmSubsystem.getAngularVelocityRPM(), 1200, DELTA);
  }

  @Test
  public void setArmFromPIDRoutine() {
    SetArmFromPID setArmFromPIDCommand1 = new SetArmFromPID(armSubsystem, 600);
    // use assert true to compare initialized value with actual value
    setArmFromPIDCommand1.schedule();
    runScheduler(0);
    DriverStationSim.setEnabled(true);
    assertEquals(
        "Set angular velocity to 600 RPM", armSubsystem.getAngularVelocityRPM(), 600, DELTA);
    wait(1000);

    SetArmFromPID setArmFromPIDCommand2 = new SetArmFromPID(armSubsystem, 0);
    setArmFromPIDCommand2.schedule();
    runScheduler();
    assertEquals("Set angular velocity to 0 RPM", armSubsystem.getAngularVelocityRPM(), 0, DELTA);
    wait(1000);

    SetArmFromPID setArmFromPIDCommand3 = new SetArmFromPID(armSubsystem, 1200);
    setArmFromPIDCommand3.schedule();
    runScheduler();
    assertEquals(
        "Set angular velocity to 1200 RPM", armSubsystem.getAngularVelocityRPM(), 1200, DELTA);
    setArmFromPIDCommand3.end();
  }
}
