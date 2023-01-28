// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class allows you to trigger commands from an analog input on a joystick (such as the
 * triggers).
 *
 * @author James@team2168.org
 */
public class JoystickAnalogButton extends Button {

  GenericHID joystick;
  int axisNumber;
  private double THRESHOLD = 0.9;

  /**
   * Create a button for triggering commands off a joystick's analsog axis
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param axisNumber The axis number
   */
  public JoystickAnalogButton(GenericHID joystick, int axisNumber) {
    this.joystick = joystick;
    this.axisNumber = axisNumber;
  }

  /**
   * Create a button for triggering commands off a joystick's analog axis
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param axisNumber The axis number
   * @param threshold The threshold to trigger above (positive) or below (negative)
   */
  public JoystickAnalogButton(GenericHID joystick, int axisNumber, double threshold) {
    this.joystick = joystick;
    this.axisNumber = axisNumber;
    THRESHOLD = threshold;
  }

  /**
   * Set the value above which triggers should occur (for positive thresholds) or below which
   * triggers should occur (for negative thresholds) The default threshold value is 0.5
   *
   * @param threshold the threshold value (1 to -1)
   */
  public void setThreshold(double threshold) {
    THRESHOLD = threshold;
  }

  /**
   * Get the defined threshold value.
   *
   * @return the threshold value
   */
  public double getThreshold() {
    return THRESHOLD;
  }

  public boolean get() {
    if (THRESHOLD < 0) {
      return joystick.getRawAxis(axisNumber)
          < THRESHOLD; // Return true if axis value is less than negative threshold
    } else {
      double b = joystick.getRawAxis(axisNumber);
      return b > THRESHOLD; // Return true if axis value is greater than positive threshold
    }
  }
}
