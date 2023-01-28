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

  ArmSubsystem armSubsystem;

  @Before
  public static void setup() {
    assert HAL.initialize(500, 0); // Initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    new ArmSubsystem();
  }

  // Subsystem tests
  @Test
  public void setInputVoltageTo0() {
    ArmSubsystem setInputVoltageTo0Command = new ArmSubsystem(0.0);
    ArmSubsystem armSubsystem;
    assertEquals("Set input voltage to 0", ArmSubsystem.getVoltage(), 0.0, DELTA);
  }

  // SetArmFromVoltage tests
  @Test
  public void setVoltageTo5Volts() {
    SetArmFromVoltage setVoltageTo5Command = new SetArmFromVoltage(armSubsystem, 5.5);
    SetArmFromVoltage setArmFromVoltage;
    assertEquals("Set voltage to 0", ArmSubsystem.getVoltage(), 5.0, DELTA);
  }

  @Test
  public void setVoltageRoutine() {
    SetArmFromVoltage setVoltageRoutineCommand = new SetArmFromVoltage(armSubsystem, 0.5);
    SetArmFromVoltage setArmFromVoltage;
    assertEquals("Set voltage to 5", ArmSubsystem.getVoltage(), 5.0, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageRoutineCommand2 = new SetArmFromVoltage(armSubsystem, 0.0);
    SetArmFromVoltage setArmFromVoltage2;
    assertEquals("Set voltage to 0", ArmSubsystem.getVoltage(), 0.0, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageRoutineCommand3 = new SetArmFromVoltage(armSubsystem, 0.5);
    SetArmFromVoltage setArmFromVoltage3;
    assertEquals("Set voltage to 5", ArmSubsystem.getVoltage(), 5.0, DELTA);
    wait(1000);
  }

  // SetArmFromPID
  @Test
  public void setArmFromPID() {
    SetArmFromPID setArmFromPIDCommand = new SetArmFromPID(armSubsystem, 1200);
    SetArmFromPID setArmFromPID;
    assertEquals(
        "Set angular velocity to 1200 RPM", ArmSubsystem.getAngularVelocityRPM(), 1200, DELTA);
  }

  @Test
  public void setArmFromPIDRoutine() {
    SetArmFromPID setArmFromPIDCommand1 = new SetArmFromPID(armSubsystem, 600);
    // use assert true to compare initialized value with actual value
    SetArmFromPID setArmFromPID1;
    assertEquals(
        "Set angular velocity to 600 RPM", armSubsystem.getAngularVelocityRPM(), 600, DELTA);
    wait(1000);

    SetArmFromPID setArmFromPIDCommand2 = new SetArmFromPID(armSubsystem, 0);
    SetArmFromPID setArmFromPID2;
    assertEquals("Set angular velocity to 0 RPM", armSubsystem.getAngularVelocityRPM(), 0, DELTA);
    wait(1000);

    SetArmFromPID setArmFromPIDCommand3 = new SetArmFromPID(armSubsystem, 1200);
    SetArmFromPID setArmFromPID3;
    assertEquals(
        "Set angular velocity to 1200 RPM", armSubsystem.getAngularVelocityRPM(), 1200, DELTA);
    setArmFromPIDCommand3.end();
  }
}
