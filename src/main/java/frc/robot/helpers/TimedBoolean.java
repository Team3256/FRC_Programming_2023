// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

public class TimedBoolean {
  private BooleanSupplier condition;
  private Timer timer = new Timer();
  private double triggerThreshold;

  public TimedBoolean(BooleanSupplier condition, double triggerThreshold) {
    this.condition = condition;
    this.triggerThreshold = triggerThreshold;
  }

  public void initialize() {
    timer.stop();
    timer.reset();
  }

  public void update() {
    if (condition.getAsBoolean()) {
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
  }

  public boolean hasBeenTrueForThreshold() {
    return timer.get() > triggerThreshold && condition.getAsBoolean();
  }
}
