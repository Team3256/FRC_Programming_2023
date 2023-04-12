// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import static frc.robot.led.LEDConstants.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;

public class LED extends SubsystemBase {
  private static final CANdle candle = new CANdle(kLedID);

  public LED() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public enum LEDSegment {
    MainStrip(0, 100, 0);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue(), 0, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(Color.black);
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              color.getRed(),
              color.getGreen(),
              color.getBlue(),
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
    }

    public void setColorFlowAnimation(Color color) {
      setAnimation(new ColorFlowAnimation(color.getRed(), color.getGreen(), color.getBlue()));
    }

    public void setFireAnimation() {
      setAnimation(new FireAnimation());
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.getRed(),
              color.getGreen(),
              color.getBlue(),
              0,
              speed,
              segmentSize,
              startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.getRed(),
              color.getGreen(),
              color.getRGB(),
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.getRed(),
              color.getGreen(),
              color.getBlue(),
              0,
              speed,
              segmentSize,
              startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }
  }
}
