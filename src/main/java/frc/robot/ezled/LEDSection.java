// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled;

import static frc.robot.Constants.LEDConstants.kResolution;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.ezled.patternBases.LEDPattern;
import frc.robot.ezled.patterns.OffPattern;

public class LEDSection {
  private final int start;
  private final int length;
  private LEDPattern ledPattern;

  /**
   * Section of LEDs that will be controlled to display a pattern
   *
   * @param start first id of the led in the section
   * @param end last id of the led in the section
   */
  public LEDSection(int start, int end) {
    this.start = start;
    length = end - start + 1;
    ledPattern = new OffPattern();
  }

  /** set the LED Pattern that the section will display */
  public void setLEDPattern(LEDPattern pattern) {
    this.ledPattern = pattern;
  }

  /**
   * Update AddressableLED's buffer with LEDSection's buffer
   *
   * @param buffer AddressableLEDBuffer that will be written to
   */
  public void writeToBuffer(AddressableLEDBuffer buffer) {
    // get LEDPattern current pattern
    ledPattern.updatePattern();
    // loop through every pixel in the pattern
    for (int pixel = 1; pixel <= kResolution; pixel++) {
      // get the LED Id corresponding to the current pixel
      int ledId = start + (length * pixel / kResolution);
      // protect ledId from out of bounds errors
      if (ledId < 0 || ledId >= buffer.getLength()) continue;
      // set the ledId on the buffer to the color of pixel
      Color color = ledPattern.getPixel(pixel);
      if (color == null) color = new Color(0, 0, 0);
      buffer.setLED(ledId, color);
    }
  }
}
