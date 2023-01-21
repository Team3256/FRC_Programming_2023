package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private AddressableLED LEDSubsystem;
    private AddressableLEDBuffer LEDBuffer;

    public LED(int length, int port){
        LEDSubsystem = new AddressableLED(port);
        LEDBuffer = new AddressableLEDBuffer(length);
        LEDSubsystem.setLength(LEDBuffer.getLength());

        LEDSubsystem.setData(LEDBuffer);
        LEDSubsystem.start();
    }

    public void setAll(int R, int G, int B){
        for (int i=0;i<LEDBuffer.getLength();i++){
            LEDBuffer.setRGB(i,R,G,B);
        }
        LEDSubsystem.setData(LEDBuffer);
    }
    int rainbowFirstPixelHue = 0;
    public void rainbow(){
        for (int i=0;i< LEDBuffer.getLength();i++){
            int hue = (rainbowFirstPixelHue + (i*180/ LEDBuffer.getLength()))%180;
            LEDBuffer.setHSV(i,hue,255,128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        LEDSubsystem.setData(LEDBuffer);
    }
    public void off(){
        for (int i=0;i<LEDBuffer.getLength();i++){
            LEDBuffer.setRGB(i,0,0,0);
        }
        LEDSubsystem.setData(LEDBuffer);
    }
}
