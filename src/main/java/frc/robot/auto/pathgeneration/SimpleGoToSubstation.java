// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueTopDoubleSubstationPose;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.led.patterns.SuccessBlinkingPattern;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class SimpleGoToSubstation {
  public static Command run(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      BooleanSupplier isCurrentPieceCone) {
    // setup
    System.out.println("Running: Go to substation from " + swerveDrive.getPose());

    Pose2d sink = kBlueTopDoubleSubstationPose;
    double preSinkDistance = Units.feetToMeters(8);
    Pose2d preSink = new Pose2d(sink.getX() - preSinkDistance, sink.getY(), sink.getRotation());

    if (DriverStation.getAlliance() == Alliance.Red) {
      sink = PathUtil.flip(sink);
      preSink = PathUtil.flip(preSink);
    }

    // commands that will be run sequentially
    Command moveToPreSink = PathGeneration.run(swerveDrive.getPose(), preSink, swerveDrive);
    Command moveArmElevatorToPreset = new ParallelCommandGroup(
        new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.DOUBLE_SUBSTATION),
        new ConditionalCommand(
            new SetArmAngle(armSubsystem, Arm.ArmPosition.DOUBLE_SUBSTATION_CONE),
            new SetArmAngle(armSubsystem, Arm.ArmPosition.DOUBLE_SUBSTATION_CUBE),
            isCurrentPieceCone));

    Command runIntake = new ConditionalCommand(
        new IntakeCone(intakeSubsystem), new IntakeCube(intakeSubsystem), isCurrentPieceCone);
    Command moveToSubstation = PathGeneration.run(preSink, sink, swerveDrive);
    Command stopIntake = new IntakeOff(intakeSubsystem);
    Command stowArmElevator = new StowArmElevator(elevatorSubsystem, armSubsystem);
    LEDSetAllSectionsPattern signalLED = new LEDSetAllSectionsPattern(ledSubsystem, new SuccessBlinkingPattern());
    Command moveAwayFromSubstation = PathGeneration.run(sink, preSink, swerveDrive);

    // return sequential of all above commands
    Command finalCommand = Commands.sequence(
        moveToPreSink,
        Commands.deadline(runIntake.withTimeout(8), moveArmElevatorToPreset, moveToSubstation),
        Commands.deadline(moveAwayFromSubstation, stowArmElevator, stopIntake, signalLED));
    return finalCommand;
  }
}
