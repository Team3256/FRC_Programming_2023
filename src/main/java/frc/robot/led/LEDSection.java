// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import static frc.robot.led.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.patternBases.LEDPattern;
import frc.robot.led.patterns.OffPattern;

public class LEDSection {
  private final int start;
  private final int length;
  private LEDPattern ledPattern;

  public LEDSection(int start, int end) {
    this.start = start;
    length = end - start + 1;
    ledPattern = new OffPattern();
  }

  // set the section's ledPattern to the one that will be displayed
  public void setLEDPattern(LEDPattern pattern) {
    this.ledPattern = pattern;
  }

  public void writeToBuffer(AddressableLEDBuffer buffer) {
    // get LEDPattern current pattern
    ledPattern.update();
    // loop through every pixel in the pattern
    for (int pixel = 1; pixel <= kResolution; pixel++) {
      // LED Id corresponding to current pixel
      int ledId = start + (length * pixel / kResolution);
      // protect from out of bounds errors
      if (ledId < 0 || ledId >= buffer.getLength()) continue;
      // set the buffer ledId to the corresponding pixel's color
      Color color = ledPattern.getPixel(pixel);
      if (color == null) color = new Color(0, 0, 0);
      buffer.setLED(ledId, color);
    }
  }
}
