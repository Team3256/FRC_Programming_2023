// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import java.util.Objects;

public class CanDeviceId {
  private final int mDeviceNumber;
  private final String mBus;

  public CanDeviceId(int deviceNumber, String bus) {
    mDeviceNumber = deviceNumber;
    mBus = bus;
  }

  // Use the default bus name (empty string).
  public CanDeviceId(int deviceNumber) {
    this(deviceNumber, "");
  }

  public int getDeviceNumber() {
    return mDeviceNumber;
  }

  public String getBus() {
    return mBus;
  }

  public boolean equals(CanDeviceId other) {
    return other.mDeviceNumber == mDeviceNumber && Objects.equals(other.mBus, mBus);
  }
}
