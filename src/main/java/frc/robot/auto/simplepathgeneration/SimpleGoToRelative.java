package frc.robot.auto.simplepathgeneration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;


public class SimpleGoToRelative {
    static Command run(SwerveDrive swerveDrive, Translation2d translation2d){
        return SimpleGoToAbsolute.run(swerveDrive, swerveDrive.getPose().plus(new Transform2d(translation2d, new Rotation2d(0))));
    }
}