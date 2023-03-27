package frc.robot.helpers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

public class TimedBoolean {
  private BooleanSupplier condition;
  private Timer timer;
  private boolean timerStarted;
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
      timer.reset();
      timer.stop();
    }
  }

  public boolean hasBeenTrueForThreshold() {
    return timer.get() > triggerThreshold && condition.getAsBoolean();
  }
}
