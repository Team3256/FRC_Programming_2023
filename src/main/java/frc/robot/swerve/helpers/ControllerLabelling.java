// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.net.URL;
import javax.imageio.ImageIO;

public class ControllerLabelling {
  public static void main(String[] args) throws Exception {
    final BufferedImage image =
        ImageIO.read(new URL("https://m.media-amazon.com/images/I/41LO2OX6pRL.jpg")); // URL here

    Graphics g = image.getGraphics();
    g.setFont(g.getFont().deriveFont(16f));

    g.drawString("LeftBumper: Auto Align Turrent & Hood\nLeft Trigger: Exhaust/Unjam", -230, 400); 
    // For left bumper/trigger
    g.drawString("Right Bumper: Rev Up Flywheel\nRight Trigger: Intake/Unjam", -230, 400); 
    // For right bumper/trigger
    g.drawString("Swerve Translation\n(Left Stick)", -230, 170); 
    // For left stick
    g.drawString("Swerve Rotation\n(Right Stick)", 122, 32); 
    // For right stick
    g.drawString("A: Reset Gyro\nB: Unassigned\nX: Toggle Azimuth Control\nY: Unassigned", 237, 168 ); 
    // For buttons A, B, X, Y
    g.drawString("Up: Feeder Manual Forward\nDown: Feeder Mannual Back\nLeft: Unassigned\nRight: Unassigned", -119, 22); 
    // For D-pad direction buttons
    g.drawString("Intake Raise\n(Back Button)", 0, 0); // Button does not exist on controller
    // For back button
    g.drawString("Intake Drop\n(Start Button", 0, 0); // Button does not exist on controller
    // For start button
    
    g.dispose();

    ImageIO.write(image, "png", new File("test.png"));
  }
}
