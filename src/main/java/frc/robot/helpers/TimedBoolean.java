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
  private final Timer timer = new Timer();
  private final double triggerThreshold;

  public TimedBoolean(BooleanSupplier condition, double triggerThreshold) {
    this.condition = condition;
    this.triggerThreshold = triggerThreshold;
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
    return timer.hasElapsed(triggerThreshold) && condition.getAsBoolean();
  }
}
