package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.swerve.SwerveConstants.AlphaOffsets;
import static frc.robot.swerve.SwerveConstants.ZipTideOffsets;


public enum RobotType {
    ZIPTIDE {
        @Override
        public Rotation2d getOffset(int module){
            return ZipTideOffsets[module];
        }
    },

    ALPHA {
        @Override
        public Rotation2d getOffset(int module){
            return AlphaOffsets[module];
        }
    };

    public abstract Rotation2d getOffset(int module);
}
