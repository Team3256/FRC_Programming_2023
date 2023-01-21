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
    g.setFont(g.getFont().deriveFont(18f));
    g.drawString("Swerve Translation\n(Left Stick)", -115, 87); // for left stick
    g.drawString("Swerve Rotation\n(Right Stick)", 62, 18); // for right stick
    g.drawString("Reset Gyro\n(A)", 117, 55); // for A button
    g.dispose();

    ImageIO.write(image, "png", new File("test.png"));
  }
}
