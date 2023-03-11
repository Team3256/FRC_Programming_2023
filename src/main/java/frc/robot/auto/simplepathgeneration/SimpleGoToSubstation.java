package frc.robot.auto.simplepathgeneration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.auto.dynamicpathgeneration.helpers.GeometryUtil;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.DynamicPathGenSuccessPattern;
import frc.robot.swerve.SwerveDrive;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueTopDoubleSubstationPose;
public class SimpleGoToSubstation {
    static Command run(SwerveDrive swerveDrive, LED ledSubsystem, boolean isCurrentPieceCone){
        Pose2d sink = DriverStation.getAlliance()==Alliance.Blue?kBlueTopDoubleSubstationPose : PathUtil.flip(kBlueTopDoubleSubstationPose);
        Command moveCommand = SimpleGoToAbsolute.run(swerveDrive, sink);
        Command intakeCommand = new ParallelCommandGroup(
            new ConditionalCommand(
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CONE_HIGH),
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CUBE_HIGH),
                ()->isCurrentPieceCone),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPosition.CONE_HIGH),
                new SetArmAngle(armSubsystem, ArmPosition.CUBE_HIGH),
                ()->isCurrentPieceCone));
        Command moveBackCommand = SimpleGoToRelative.run(swerveDrive, new Translation2d(0,-0.25));
        LEDSetAllSectionsPattern flashCommand = new LEDSetAllSectionsPattern(ledSubsystem, new DynamicPathGenSuccessPattern());

        Command finalCommand = new SequentialCommandGroup(moveCommand, intakeCommand, moveBackCommand, flashCommand);
        return finalCommand;
    }
}