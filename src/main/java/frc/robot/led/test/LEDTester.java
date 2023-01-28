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

/** Simple LED class to test LED is working */
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

  public void setAll(int R, int G, int B) {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, R, G, B);
    }
    LED.setData(LEDBuffer);
  }

  int rainbowFirstPixelHue = 0;

  public void rainbow() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / LEDBuffer.getLength())) % 180;
      LEDBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    LED.setData(LEDBuffer);
  }

  public void off() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
    LED.setData(LEDBuffer);
  }
}
