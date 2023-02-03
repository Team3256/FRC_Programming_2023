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

  double tH = 0;
  int iH = 100;
  int dH = 50;
  int flip = 1;

  public void gradientBlue() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      double hue = iH + (dH + tH + flip * 0.7 * i * dH / LEDBuffer.getLength()) % dH;
      LEDBuffer.setHSV(i, (int) hue, 255, 255);
    }
    tH += 0.3 * flip;
    tH = (tH + dH) % dH;
    if (flip == 1 && tH > 49) flip *= -1;
    if (flip == -1 && tH < 1) flip *= -1;
    LED.setData(LEDBuffer);
  }

  //  int iH = 208;
  //  int iS = 87;
  //  int iV = 30;
  //  int fH = 0;
  //  int fS = 0;
  //  int fV = 100;
  //  int kT = 60;
  //  int t = 0;
  //
  //  public void spirit() {
  //    t = (t + 1) % kT;
  //
  //    double mH = (double) (fS - iS) / kT;
  //    double mS = (double) (fS - iS) / kT;
  //    double mV = (double) (fV - iV) / kT;
  //
  //    int h = (int) (iH + mH * t);
  //    int s = (int) (iS + mS * t);
  //    int v = (int) (iV + mV * t);
  //
  //    for (int i = 0; i < LEDBuffer.getLength(); i++) {
  //      LEDBuffer.setHSV(i, h, s, v);
  //    }
  //    LED.setData(LEDBuffer);
  //  }

  public void off() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, 0, 0, 0);
    }
    LED.setData(LEDBuffer);
  }
}
