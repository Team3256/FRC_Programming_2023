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
import frc.robot.led.patterns.OffPattern;

/** displays a pattern onto section of LED */
public class LEDSection {
  private final int start;
  private final int length;
  private LEDPattern pattern;

  public LEDSection(int start, int end) {
    this.start = start;
    length = end - start + 1;
    pattern = new OffPattern();
  }

  public void setPattern(LEDPattern pattern) {
    this.pattern = pattern;
  }

  public void writeToBuffer(AddressableLEDBuffer buffer) {
    pattern.update();
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
