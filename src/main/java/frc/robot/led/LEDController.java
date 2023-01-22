// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.drivers.Color;
import frc.robot.led.patterns.CubePattern;
import frc.robot.led.patterns.LEDPattern;
import java.util.ArrayList;

/**
 * class that allocates sections of a full LED strip to multiple strips that display the same
 * information information is stored in percentage instead of pixel ledContainers stores the
 * different sections of the full LED strip ledPatterns stores the different patterns that are
 * displayed totalPattern is the information from ledPatterns concatenated led and buffer are
 * standard for controlling the led periodic will be called in Robot periodic it updates
 * totalPattern, updates ledContainers, and then sets the led
 */
public class LEDController {
  ArrayList<LEDContainer> ledContainers = new ArrayList<>();
  ArrayList<LEDPattern> ledPatterns = new ArrayList<>();
  Color[] totalPattern = new Color[100];
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public LEDController() {
    // initialize led and buffer
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(75);
    led.setLength(buffer.getLength());

    // initialize led containers sequentially
    ledContainers.add(new LEDContainer(0, 24));
    ledContainers.add(new LEDContainer(25, 49));
    ledContainers.add(new LEDContainer(50, 74));
    ledContainers.add(new LEDContainer(75, 99));

    // initialize the total pattern percentage array
    for (int i = 0; i < 100; i++) {
      totalPattern[i] = new Color(0, 0, 0);
    }

    // initialize the patterns used
    ledPatterns.add(new CubePattern(totalPattern));

    // turn on led to default
    led.setData(buffer);
    led.start();
  }

  public void periodic() {
    // concatenate ledPatterns into totalPattern
    for (LEDPattern pattern : ledPatterns) {
      pattern.update();
    }
    // set the buffer to display totalPattern in each container
    for (LEDContainer container : ledContainers) {
      container.display(totalPattern, buffer);
    }
    // set the led
    led.setData(buffer);
  }
}
