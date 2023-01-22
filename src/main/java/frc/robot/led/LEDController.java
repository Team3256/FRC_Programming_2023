package frc.robot.led;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.drivers.Color;
import frc.robot.led.patterns.LEDPattern;

/**
 * class that allocates sections of a full LED strip
 * to multiple strips that display the same information
 * information is stored in percentage instead of pixel
 *
 * ledContainers stores the different sections of the full LED strip
 * ledPatterns stores the different patterns that are displayed
 * totalPattern is the information from ledPatterns concatenated
 *
 * led and buffer are standard for controlling the leds
 *
 * periodic will be called in Robot periodic
 * it updates totalPattern, updates ledContainers, and then sets the leds
 */
public class LEDController {
    ArrayList<LEDContainer> ledContainers = new ArrayList<>();
    ArrayList<LEDPattern> ledPatterns = new ArrayList<>();
    Color[] totalPattern = new Color[100];
    AddressableLED led;
    AddressableLEDBuffer buffer;

    public LEDController(){
        //initialize led and buffer
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(75);
        led.setLength(buffer.getLength());

        //initialize led containers sequentially
        ledContainers.add(new LEDContainer(0, 24));
        ledContainers.add(new LEDContainer(25, 49));
        ledContainers.add(new LEDContainer(50, 74));
        ledContainers.add(new LEDContainer(75, 99));

        //initialize the total pattern percentage array
        for (int i=0;i<100;i++){
            totalPattern[i] = new Color(0,0,0);
        }

        //initialize the patterns used
        ledPatterns.add(new BallColorPattern(totalPattern));

        //turn on leds to default
        led.setData(buffer);
        led.start();
    }

    public void periodic(){
        //concatenate ledPatterns into totalPattern
        for (LEDPattern pattern : ledPatterns){
            pattern.update();
        }
        //set the buffer to display totalPattern in each container
        for (LEDContainer container : ledContainers){
            container.display(totalPattern, buffer);
        }
        //set the leds
        led.setData(buffer);
    }
}