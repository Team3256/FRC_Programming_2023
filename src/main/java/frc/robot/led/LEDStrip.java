// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.led.patternBases.LEDPattern;
import java.util.Arrays;

public class LEDStrip extends SubsystemBase {
  private final int length;
  private final int sections;
  private final LEDSection[] LEDSections;
  private final AddressableLED addressableLED;
  private final AddressableLEDBuffer buffer;

  /**
   * Addressable LED strip controller.
   *
   * @param port PWM port on the RoboRio
   * @param LEDSectionLengths An array of LED section lengths
   */
  public LEDStrip(int port, int[] LEDSectionLengths) {
    length = Arrays.stream(LEDSectionLengths).sum();
    sections = LEDSectionLengths.length;
    LEDSections = new LEDSection[sections];
    addressableLED = new AddressableLED(port);
    addressableLED.setLength(length);
    buffer = new AddressableLEDBuffer(length);

    // initialize LED sections
    int cur = 0;
    for (int i = 0; i < sections; i++) {
      LEDSections[i] = new LEDSection(cur, cur + LEDSectionLengths[i] - 1);
      cur += LEDSectionLengths[i];
    }

    // start LEDs
    addressableLED.start();
    addressableLED.setData(buffer);
  }

  public void set(int sectionId, LEDPattern pattern) {
    // set a specific section's buffer to a pattern
    LEDSections[sectionId].setLEDPattern(pattern);
  }

  public void setAll(LEDPattern pattern) {
    // set each section's buffer to the same pattern
    for (int i = 0; i < sections; i++) {
      set(i, pattern);
    }
  }

  public void periodic() {
    // set each container's display to the pattern in it's buffer
    for (LEDSection section : LEDSections) {
      section.writeToBuffer(buffer);
    }
    addressableLED.setData(buffer);
  }
}
