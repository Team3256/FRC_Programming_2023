// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

public class TimedBoolean {
  private BooleanSupplier condition;
  private Timer timer = new Timer();
  private boolean timerStarted = false;
  private double triggerThreshold;

  public TimedBoolean(BooleanSupplier condition, double triggerThreshold) {
    this.condition = condition;
    this.triggerThreshold = triggerThreshold;
  }

  public void update() {
    if (condition.getAsBoolean() && !timerStarted) {
      timer.start();
      timerStarted = true;
    }
    if (!condition.getAsBoolean()) {
      timer.stop();
      timer.reset();
      timerStarted = false;
    }
    SmartDashboard.putNumber("Timer", timer.get());
  }

  public boolean hasBeenTrueForThreshold() {
    return timer.get() > triggerThreshold && condition.getAsBoolean();
  }
}
