// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led.test;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple bare-bones LED_Old class to test if your Addressable LED_Old is working Call turnOn in
 * robotInit and turnOff in robotDisabled
 */
public class LEDTester extends SubsystemBase {
  private final AddressableLED LED;
  private final AddressableLEDBuffer LEDBuffer;

  public LEDTester(int length, int port) {
    LED = new AddressableLED(port);
    LEDBuffer = new AddressableLEDBuffer(length);
    LED.setLength(length);
    LED.setData(LEDBuffer);
    LED.start();
  }

  public void turnOn() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 255, 255, 255);
    }
    LED.setData(LEDBuffer);
  }

  public void turnOff() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
    LED.setData(LEDBuffer);
  }
}
