// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping
 * duplicate set commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
  protected double mLastSet = Double.NaN;
  protected TalonFXControlMode mLastControlMode = null;

  public LazyTalonFX(CanDeviceId id) {
    super(id.getDeviceNumber(), id.getBus());
  }

  public double getLastSet() {
    return mLastSet;
  }

  @Override
  public void set(TalonFXControlMode mode, double value) {
    if (value != mLastSet || mode != mLastControlMode) {
      mLastSet = value;
      mLastControlMode = mode;
      super.set(mode, value);
    }
  }
}
