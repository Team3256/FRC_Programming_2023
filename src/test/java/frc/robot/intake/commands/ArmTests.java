// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.intake.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.commands.SetArmFromVoltage;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ArmTests {
  private static ArmSubsystem armSubsystem;
  private double DELTA = 0.05;

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // Initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
    armSubsystem = new ArmSubsystem();
  }

  @Test
  public void setVoltage5() throws InterruptedException {
    armSubsystem = new ArmSubsystem();
    DoubleSupplier setVoltage = () -> 5.0;
    SetArmFromVoltage setVoltageBackTo5Volts = new SetArmFromVoltage(armSubsystem, setVoltage);
    setVoltageBackTo5Volts.execute();
    runScheduler(1);
    double voltage = armSubsystem.getVoltage();
    assertEquals(5.0, voltage, DELTA);
  }

  @Test
  public void setVoltage0() throws InterruptedException {
    armSubsystem = new ArmSubsystem();
    DoubleSupplier setVoltage = () -> 0.0;
    SetArmFromVoltage setVoltageTo0Volts = new SetArmFromVoltage(armSubsystem, setVoltage);
    setVoltageTo0Volts.execute();
    runScheduler(1);
    double voltage = armSubsystem.getVoltage();
    assertEquals(0.0, voltage, DELTA);
  }

  // SetArmFromVoltage test (one routine)
  @Test
  public void setVoltageRoutine5() throws InterruptedException {
    DoubleSupplier setVoltage = () -> 0.0;
    SetArmFromVoltage setVoltageTo0Volts = new SetArmFromVoltage(armSubsystem, setVoltage);
    setVoltageTo0Volts.execute();
    runScheduler(1);
    double voltage = armSubsystem.getVoltage();
    assertEquals(0.0, voltage, DELTA);
    Thread.sleep(1000);

    setVoltage = () -> 5.0;
    SetArmFromVoltage setVoltageBackTo5Volts = new SetArmFromVoltage(armSubsystem, setVoltage);
    setVoltageBackTo5Volts.execute();
    runScheduler(1);
    voltage = armSubsystem.getVoltage();
    assertEquals(5.0, voltage, DELTA);
  }

  //  @Test
  //  public void setPIDVoltage600() throws InterruptedException {
  //    armSubsystem = new ArmSubsystem();
  //    SetArmFromPID setArmPIDTo600 = new SetArmFromPID(armSubsystem, 600);
  //    setArmPIDTo600.execute();
  //    CommandScheduler.getInstance().schedule(setArmPIDTo600);
  //    runScheduler(1);
  //    double angularVelocity = armSubsystem.getAngularVelocityRPM();
  //    assertEquals(600, angularVelocity, DELTA);
  //  }
  // SetArmFromPID test (one routine)

  //  @Test
  //    public void setArmFromPIDRoutine() throws InterruptedException {
  //      SetArmFromPID setArmPIDTo600 = new SetArmFromPID(armSubsystem, 600);
  //      setArmPIDTo600.schedule();
  //      runScheduler(1.5);
  //      assertEquals(
  //          "Set angular velocity to 600 RPM", armSubsystem.getAngularVelocityRPM(), 600, DELTA);
  //      wait(1000);
  //
  //      SetArmFromPID setArmPIDTo0 = new SetArmFromPID(armSubsystem, 0);
  //      setArmPIDTo0.schedule();
  //      runScheduler(1);
  //      assertEquals("Set angular velocity to 0 RPM", armSubsystem.getAngularVelocityRPM(), 0,
  //   DELTA);
  //      wait(1000);
  //
  //      SetArmFromPID setArmPIDTo1200 = new SetArmFromPID(armSubsystem, 1200);
  //      setArmPIDTo1200.schedule();
  //      runScheduler(1.5);
  //      assertEquals(
  //          "Set angular velocity to 1200 RPM", armSubsystem.getAngularVelocityRPM(), 1200,
  // DELTA);
  //      // setArmPIDTo1200.end();
  //    }

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
