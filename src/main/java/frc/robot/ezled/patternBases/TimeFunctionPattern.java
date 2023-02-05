// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.patternBases;

import static frc.robot.Constants.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.util.Color;

public abstract class TimeFunctionPattern extends LEDPattern {
  // number of times periodic loop is run, equivalent of 20 ms
  private int ticks = 0;
  // determines whether to render and animate the color as rgb or hsv
  boolean rgb;

  /**
   * LED Pattern that animates as a function of time
   *
   * @param rgb determines whether to render the colors as rgb or hsv
   */
  public TimeFunctionPattern(boolean rgb) {
    this.rgb = rgb;
  }
  /** Calculate the Red/Hue [0,1) value of an LED as a function of ticks */
  protected abstract double calculateA(int ticks);
  /** Calculate the Green/Saturation [0,1) value of an LED as a function of ticks */
  protected abstract double calculateB(int ticks);
  /** Calculate the Blue/Value [0,1) value of an LED as a function of ticks */
  protected abstract double calculateC(int ticks);

  /** Update the pattern of LED Pattern based on time */
  @Override
  public void updatePattern() {
    for (int pixel = 1; pixel <= kResolution; pixel++) {
      double A = calculateA(ticks + pixel);
      double B = calculateB(ticks + pixel);
      double C = calculateC(ticks + pixel);

      if (rgb) {
        setPixel(pixel, new Color(A, B, C));
      } else {
        setPixel(pixel, Color.fromHSV((int) (A * 360), (int) (B * 255), (int) (C * 255)));
      }
    }
    ticks++;
  }
}
