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
import frc.robot.led.patterns.BlinkingConePattern;
import frc.robot.led.patterns.BlinkingCubePattern;
import java.util.Arrays;

public class LED extends SubsystemBase {
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
  public LED(int port, int[] LEDSectionLengths) {
    // initialize object
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

    // start the LEDs
    addressableLED.start();
    addressableLED.setData(buffer);
  }

  // set a specific section's buffer to a LEDPattern
  public void set(int sectionId, LEDPattern ledPattern) {
    LEDSections[sectionId].setLEDPattern(ledPattern);
  }

  // set each section's buffer to the same LEDPattern
  public void setAll(LEDPattern ledPattern) {
    for (int i = 0; i < sections; i++) {
      set(i, ledPattern);
    }
  }

  /** set each container's display to the pattern in it's buffer */
  public void periodic() {
    for (LEDSection section : LEDSections) {
      section.writeToBuffer(buffer);
    }
    addressableLED.setData(buffer);
  }

  private boolean isCubePiece = true;

  /** Toggle whether a cone or cube pattern will be displayed */
  public void toggleGamePiece() {
    if (isCubePiece) {
      setAll(new BlinkingCubePattern());
    } else {
      setAll(new BlinkingConePattern());
    }
    isCubePiece = !isCubePiece;
  }
}
