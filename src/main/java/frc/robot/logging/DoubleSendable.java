// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.DoubleSupplier;

public class DoubleSendable implements Sendable {
  DoubleSupplier doubleSupplier;
  String dashboardType = "TextView";

  public DoubleSendable(DoubleSupplier doubleSupplier) {
    this.doubleSupplier = doubleSupplier;
  }

  public DoubleSendable(DoubleSupplier doubleSupplier, String dashboardType ) {
    this(doubleSupplier);
    this.dashboardType = dashboardType;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(dashboardType);
    builder.addDoubleProperty("Value", doubleSupplier, null);
  }
}
