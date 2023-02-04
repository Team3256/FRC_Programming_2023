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
import java.util.ArrayList;
import java.util.Arrays;

/**
 * allocates sections of a full LED strip to multiple strips that display the same image information
 * is stored in percentage instead of pixel ledContainers stores the different sections of the full
 * LED strip ledPatterns stores the different patterns that are displayed totalPattern is the
 * information from ledPatterns concatenated led and buffer are standard for controlling the led
 * periodic will be called in Robot periodic it updates totalPattern, updates ledContainers, and
 * then sets the led
 */
public class LEDStrip extends SubsystemBase {
  private final int length;
  private final int sections;
  private final LEDSection[] ledSections;
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  /** 
   * Addressable LED strip controller.
   * @param port PWM port on the RoboRio
   * @param ledSections An array of LED section lengths
   */
  public LEDStrip(int port, int[] ledSectionLengths) {
    // initialize led and buffer
    length = Arrays.stream(ledSectionLengths).sum();
    sections = ledSectionLengths.length;
    ledSections = new LEDSection[sections];
    
    led = new AddressableLED(port);
    led.setLength(length);
    buffer = new AddressableLEDBuffer(length);

    // initialize led containers sequentially
    int cur = 0;
    for (int i=0;i<sections;i++){
      ledSections[i]=new LEDSection(cur, cur+ledSectionLengths[i]-1);
      cur+=ledSectionLengths[i];
    }

    // turn on led to default
    led.start();
    led.setData(buffer);
  }

  public void set(int sectionId, LEDPattern pattern) {
    // set a specific section's buffer to a pattern
    ledSections[sectionId].setPattern(pattern);
  }

  public void setAll(LEDPattern pattern) {
    // set each section's buffer to the same pattern
    for (int i = 0; i < sections; i++) {
      set(i, pattern);
    }
  }

  public void periodic() {
    // display the pattern in each container's buffer
    for (LEDSection section : ledSections) {
      section.writeToBuffer(buffer);
    }
    led.setData(buffer);
  }
}
