// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UnitTestBase;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ArmTests extends UnitTestBase {

  static final double DELTA = 1e-2;

  Arm armSubsystem;

  @BeforeEach
  public void setup() {
    super.setup();
    armSubsystem = new Arm();
  }

  // Arm Sim is setting to minimum angle. Tests don't really work

  @Test
  public void testSetArmAngle() {
    armAngleRoutine(new Rotation2d(0));
  }

  public void armAngleRoutine(Rotation2d... angles) {
    for (Rotation2d angle : angles) {
      Command setAngleCommand = new SetArmAngle(armSubsystem, angle);
      runScheduler(3, setAngleCommand, armSubsystem);
      assertEquals(
          angle.getRadians(),
          armSubsystem.getArmPosition(),
          DELTA,
          "Set angle to " + angle.getRadians() + " radians");
    }
    //    armSubsystem.off();
  }
}
