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
import frc.robot.led.patterns.ConePattern;
import frc.robot.led.patterns.CubePattern;
import frc.robot.led.patterns.DrivingPattern;

import java.util.ArrayList;

/**
 * allocates sections of a full LED strip to multiple strips that display the same image
 * information is stored in percentage instead of pixel
 * ledContainers stores the different sections of the full LED strip
 * ledPatterns stores the different patterns that are displayed
 * totalPattern is the information from ledPatterns concatenated
 * led and buffer are standard for controlling the led
 * periodic will be called in Robot periodic
 * it updates totalPattern, updates ledContainers, and then sets the led
 */
public class LED extends SubsystemBase{
  private final ArrayList<LEDSection> ledContainers = new ArrayList<>();
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  private final CubePattern cubePattern = new CubePattern();
  private final ConePattern conePattern = new ConePattern();
  private final DrivingPattern drivingPattern = new DrivingPattern();

  public LED(int port,int length) {
    // initialize led and buffer
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(buffer.getLength());

    // initialize led containers sequentially
    ledContainers.add(new LEDSection(0, 24));
    ledContainers.add(new LEDSection(25, 49));
    ledContainers.add(new LEDSection(50, 74));
    ledContainers.add(new LEDSection(75, 99));

    // turn on led to default
    led.start();
    led.setData(buffer);
  }

  public void set(int sectionId, LEDPattern pattern){
    ledContainers.get(sectionId).writeToBuffer(pattern,buffer);
  }

  public void bulkSet(LEDPattern pattern){
    for (int i=0;i<ledContainers.size();i++){
      set(i,pattern);
    }
  }

  public void periodic() {
    //display pattern in each container
    for (LEDSection container : ledContainers) {
      container.writeToBuffer(cubePattern, buffer);
    }
    led.setData(buffer);
  }
}
