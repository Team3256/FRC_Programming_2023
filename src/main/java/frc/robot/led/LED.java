// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.led;

import static frc.robot.Constants.ShuffleboardConstants.kDriverTabName;
import static frc.robot.Constants.ShuffleboardConstants.kLEDLayoutName;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.led.commands.LEDToggleGamePieceDisplay;
import frc.robot.logging.Loggable;

public class LED extends SubsystemBase implements Loggable {
  private final AddressableLED addressableLED;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDSim ledSim;

  private long previousTimeFlash = 0;
  private int rainbowFirstPixelHue = 30;
  private int currentTrailIndex = 0;
  private boolean isOn = false;
  private boolean isCubePiece = false;

  /**
   * Addressable LED strip controller.
   *
   * @param port PWM port on the RoboRio
   * @param LEDSectionLengths An array of LED section lengths
   */
  public LED() {
    this.addressableLED = new AddressableLED(LEDConstants.kLEDPWMPort);
    this.buffer = new AddressableLEDBuffer(LEDConstants.kNumberOfLEDs);

    this.addressableLED.setLength(this.buffer.getLength());
    this.addressableLED.setData(this.buffer);
    this.addressableLED.start();

    this.ledSim = new AddressableLEDSim(this.addressableLED);
    // SmartDashboard.putData("LED Simulator", this::getLedSim);
  }

  private byte[] getLedData() {
    return this.ledSim.getData();
  }

  public void glow(int r, int g, int b) {
    for (int i = 0; i < this.buffer.getLength(); i++) {
      this.buffer.setRGB(i, r, g, b);
    }
  }

  public void glow(Color color) {
    this.glow((int) color.red, (int) color.green, (int) color.blue);
  }

  public void rainbow() {
    for (int i = 0; i < this.buffer.getLength(); i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / this.buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 2;
    rainbowFirstPixelHue %= 180;
  }

  public void flash(int r, int g, int b) {
    long currentTime = System.currentTimeMillis();
    if ((currentTime - previousTimeFlash) >= LEDConstants.kInterval) {
      previousTimeFlash = currentTime;
      isOn = !isOn;
    }
    if (!isOn) {
      r = 0;
      g = 0;
      b = 0;
    }
    for (int i = 0; i < this.buffer.getLength(); i++) {
      this.buffer.setRGB(i, r, g, b);
    }
  }

  public void flash(Color color) {
    flash((int) color.red, (int) color.green, (int) color.blue);
  }

  public void trail(Color bgColor, Color movingColor, int trailLength) {
    glow(bgColor);
    for (int i = currentTrailIndex; i < currentTrailIndex + trailLength; i++) {
      this.buffer.setLED(i % this.buffer.getLength(), movingColor);
    }
    currentTrailIndex++;
  }

  public void toggleGamePiece() {
    this.isCubePiece = !this.isCubePiece;
  }

  public boolean getCubePiece() {
    return this.isCubePiece;
  }

  @Override
  public void periodic() {

    if (!DriverStation.isDSAttached()) {
      rainbow();
      this.addressableLED.setData(this.buffer);
      return;
    } else {
      if (DriverStation.isDisabled()) {
        glow(255, 0, 0);
        this.addressableLED.setData(this.buffer);
      } else if (DriverStation.isEnabled()) {
        if (false) {
          flash(0, 255, 0);
          this.addressableLED.setData(this.buffer);
          return;
        } else if (this.isCubePiece) {
          flash(255, 0, 0);
          this.addressableLED.setData(this.buffer);
          return;
        } else {
          flash(0, 0, 255);
          this.addressableLED.setData(this.buffer);
        }
      }
    }
  }

  @Override
  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName).add(new LEDToggleGamePieceDisplay(this));
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab).getLayout(kLEDLayoutName, BuiltInLayouts.kList).withSize(2, 2);
  }
}
