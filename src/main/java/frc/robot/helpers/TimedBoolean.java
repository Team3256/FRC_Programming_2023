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
  private final BooleanSupplier condition;
  private final Timer timer = new Timer();
  private final double triggerThresholdTime;

  public TimedBoolean(BooleanSupplier condition, double triggerThresholdTime) {
    this.condition = condition;
    this.triggerThresholdTime = triggerThresholdTime;
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
    return timer.get() > triggerThresholdTime && condition.getAsBoolean();
  }
}
