// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.simplepathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueTopDoubleSubstationPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.DynamicPathGenSuccessPattern;
import frc.robot.swerve.SwerveDrive;

public class SimpleGoToSubstation {
  public static Command run(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      boolean isCurrentPieceCone) {
    //setup
    System.out.println("Running: Go to substation from " + swerveDrive.getPose());
    Pose2d sink =
        DriverStation.getAlliance() == Alliance.Blue
            ? kBlueTopDoubleSubstationPose
            : PathUtil.flip(kBlueTopDoubleSubstationPose);
    double chargeDistance = Units.feetToMeters(4.9)/2;
    Pose2d preSink = new Pose2d(sink.getX()-chargeDistance,sink.getY(),sink.getRotation());

    //commands that will be run sequentially
    Command moveToPreSink = SimpleGoToAbsolute.run(swerveDrive, preSink);
    Command moveArmElevatorToPreset =
        new ParallelCommandGroup(
            new ConditionalCommand(
                new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.CONE_HIGH),
                new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.CUBE_HIGH),
                () -> isCurrentPieceCone),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, Arm.ArmPosition.CONE_HIGH),
                new SetArmAngle(armSubsystem, Arm.ArmPosition.CUBE_HIGH),
                () -> isCurrentPieceCone));
    Command runIntake =
        new ConditionalCommand(
            new IntakeCone(intakeSubsystem),
            new IntakeCube(intakeSubsystem),
            () -> isCurrentPieceCone);
    Command moveToSubstation = SimpleGoToAbsolute.run(swerveDrive, sink);
    Command wait = new WaitCommand(0.5);
    Command stopIntake = new IntakeOff(intakeSubsystem);
    Command stowArmElevator = new StowArmElevator(elevatorSubsystem, armSubsystem);
    LEDSetAllSectionsPattern signalLED =
        new LEDSetAllSectionsPattern(ledSubsystem, new DynamicPathGenSuccessPattern());
    
    //return sequential of all above commands
    Command finalCommand =
        new SequentialCommandGroup(
            moveToPreSink,
            moveArmElevatorToPreset,
            runIntake,
            moveToSubstation,
            wait,
            stopIntake,
            moveToPreSink,
            stowArmElevator,
            signalLED);
    return finalCommand;
  }
}
