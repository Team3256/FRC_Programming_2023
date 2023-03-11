// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.simplepathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueTopDoubleSubstationPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.DynamicPathGenSuccessPattern;
import frc.robot.swerve.SwerveDrive;

public class SimpleGoToSubstation {
  static Command run(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      boolean isCurrentPieceCone) {
    Pose2d sink =
        DriverStation.getAlliance() == Alliance.Blue
            ? kBlueTopDoubleSubstationPose
            : PathUtil.flip(kBlueTopDoubleSubstationPose);
    Command moveCommand = SimpleGoToAbsolute.run(swerveDrive, sink);
    Command moveArmAndElevatorCommand =
        new ParallelCommandGroup(
            new ConditionalCommand(
                new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.CONE_HIGH),
                new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.CUBE_HIGH),
                () -> isCurrentPieceCone),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, Arm.ArmPosition.CONE_HIGH),
                new SetArmAngle(armSubsystem, Arm.ArmPosition.CUBE_HIGH),
                () -> isCurrentPieceCone));
    Command intakeCommand =
        new ConditionalCommand(
            new IntakeCone(intakeSubsystem),
            new IntakeCube(intakeSubsystem),
            () -> isCurrentPieceCone);
    Command moveBackCommand = SimpleGoToRelative.run(swerveDrive, new Translation2d(0, -0.25));
    LEDSetAllSectionsPattern flashCommand =
        new LEDSetAllSectionsPattern(ledSubsystem, new DynamicPathGenSuccessPattern());

    Command finalCommand =
        new SequentialCommandGroup(moveCommand, intakeCommand, moveBackCommand, flashCommand);
    return finalCommand;
  }
}
