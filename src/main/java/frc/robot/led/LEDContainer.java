// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.drivers.Color;

/**
 * class that properly displays a pattern percentage array onto the specified section of the whole
 * LED strip
 */
public class LEDContainer {
  int start;
  int end;
  int length;

  public LEDContainer(int start, int end) {
    this.start = start;
    this.end = end;
    length = end - start + 1;
  }

  public void display(Color[] totalPattern, AddressableLEDBuffer buffer) {
    // loop through every percent
    for (int i = 0; i < 100; i++) {
      // convert percentage to pixel
      int pixel = start + (length * i / 100);
      // finesse out of bound errors
      if (pixel < 0 || pixel >= buffer.getLength()) continue;
      // set the buffer pixel to the color
      Color color = totalPattern[i];
      buffer.setRGB(pixel, color.R, color.G, color.B);
    }
  }
}
