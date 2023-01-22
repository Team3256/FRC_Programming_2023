// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.drivers.Color;

import frc.robot.led.patternBases.LEDPattern;

/**
 * displays a pattern onto section of LED
 */
public class LEDSection{
  int start;
  int end;
  int length;

  public LEDSection(int start,int end) {
    this.start = start;
    this.end = end;
    length = end - start + 1;
  }

  public void writeToBuffer(LEDPattern pattern,AddressableLEDBuffer buffer) {
    // loop through every percent
    for (int percent = 1; percent <= 100; percent++) {
      // convert percentage to pixel
      int pixel = start + (length * percent / 100);
      // finesse out of bound errors
      if (pixel < 0 || pixel >= buffer.getLength()) continue;
      // set the buffer pixel to the color

      Color color = pattern.get(percent);
      buffer.setRGB(pixel, color.R, color.G, color.B);
    }
  }
}
