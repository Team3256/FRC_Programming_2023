package frc.robot.swerve.helpers;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.net.URL;

public class ControllerLabelling {
    public static void main(String[] args) throws Exception {
        final BufferedImage image = ImageIO.read(new URL
                ("https://m.media-amazon.com/images/I/41LO2OX6pRL.jpg")); //URL here

        Graphics g = image.getGraphics();
        g.setFont(g.getFont().deriveFont(18f));
        g.drawString("Swerve Translation\n(Left Stick)", -115, 87); // for left stick
        g.drawString("Swerve Rotation\n(Right Stick)", 62, 18); // for right stick
        g.drawString("Reset Gyro\n(A)", 117, 55); // for A button
        g.dispose();

        ImageIO.write(image, "png", new File("test.png"));
    }
}