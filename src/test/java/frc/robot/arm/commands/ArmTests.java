// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static org.junit.jupiter.api.Assertions;

import java.beans.Transient;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UnitTestBase;
import frc.robot.arm.ArmSubsystem;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ArmTests extends UnitTestBase {

  static ArmSubsystem armSubsystem;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
  }

  @Test
  public void voltageRoutine() {
    voltageRoutine(5.0, 0.0, 5.0);
  }

  // SetArmFromVoltage test (one routine)
  @Test
  public void setVoltageRoutine(double firstSet, double secondSet, double thirdSet) {
    SetArmFromVoltage setVoltageToFirstSet = new SetArmFromVoltage(armSubsystem, firstSet);
    setVoltageToFirstSet.schedule();
    runScheduler(1);
    assertEquals("Set voltage to " + firstSet, ArmSubsystem.getVoltage(), firstSet, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageToSecondSet = new SetArmFromVoltage(armSubsystem, secondSet);
    setVoltageToSecondSet.schedule();
    runScheduler(1);
    assertEquals("Set voltage to " + secondSet, ArmSubsystem.getVoltage(), secondSet, DELTA);
    wait(1500);

    SetArmFromVoltage setVoltageToThirdSet = new SetArmFromVoltage(armSubsystem, thirdSet);
    setVoltageToThirdSet.schedule();
    runScheduler(1);
    assertEquals("Set voltage to " + thirdSet, ArmSubsystem.getVoltage(), thirdSet, DELTA);
    wait(1000);
  }

  // SetArmFromPID test (one routine)
  @Test
  public void armPIDRoutine() {
    setArmFromPIDRoutine(600, 0, 1200);
  }

  @Test
  public void setArmFromPIDRoutine(double firstSet, double secondSet, double thirdSet) {
    SetArmFromPID setArmPIDToFirstSet = new SetArmFromPID(armSubsystem, firstSet);
    setArmPIDToFirstSet.schedule();
    runScheduler(1.5);
    assertEquals(
        "Set angular velocity to " + firstSet + " RPM", armSubsystem.getAngularVelocityRPM(), firstSet, DELTA);
    wait(1000);

    SetArmFromPID setArmPIDToSecondSet = new SetArmFromPID(armSubsystem, secondSet);
    setArmPIDToSecondSet.schedule();
    runScheduler(1);
    assertEquals("Set angular velocity to " + secondSet + " RPM", armSubsystem.getAngularVelocityRPM(), secondSet, DELTA);
    wait(1000);

    SetArmFromPID setArmPIDToThirdSet = new SetArmFromPID(armSubsystem, thirdSet);
    setArmPIDToThirdSet.schedule();
    runScheduler(1.5);
    assertEquals(
        "Set angular velocity to " + thirdSet + " RPM", armSubsystem.getAngularVelocityRPM(), thirdSet, DELTA);
    setArmPIDTo1200.end();
  }
}