// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.DoubleSupplier;

public class GyroSendable implements Sendable {
  DoubleSupplier doubleSupplier;

  public GyroSendable(DoubleSupplier doubleSupplier) {
    this.doubleSupplier = doubleSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", doubleSupplier, null);
  }
}
