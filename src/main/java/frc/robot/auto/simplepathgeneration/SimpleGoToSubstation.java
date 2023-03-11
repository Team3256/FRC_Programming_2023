package frc.robot.auto.simplepathgeneration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.dynamicpathgeneration.helpers.GeometryUtil;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.swerve.SwerveDrive;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueTopDoubleSubstationPose;
public class SimpleGoToSubstation extends SimpleGoToPose {
    static Command run(SwerveDrive swerveDrive){
        Pose2d sink = DriverStation.getAlliance()==Alliance.Blue?kBlueTopDoubleSubstationPose : PathUtil.flip(kBlueTopDoubleSubstationPose);
        Command moveCommand = run(swerveDrive, sink);
        Command finalCommand = new SequentialCommandGroup(moveCommand);
        return finalCommand;
    }
}