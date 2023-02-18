package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerve;
import frc.robot.swerve.commands.TeleopSwerve;
import frc.robot.swerve.commands.TeleopSwerveLimited;
import frc.robot.swerve.commands.TeleopSwerveWithAzimuth;

import static frc.robot.Constants.*;
import static frc.robot.swerve.SwerveConstants.kFieldRelative;
import static frc.robot.swerve.SwerveConstants.kOpenLoop;

public class PitSubsystemRoutine {
    Elevator elevatorSubsystem;
    Intake intakeSubsystem;
    SwerveDrive swerveSubsystem;
    private final CommandXboxController driver = new CommandXboxController(0);

    public PitSubsystemRoutine(Elevator elevator, Intake intake, SwerveDrive swerve) {
        elevatorSubsystem = elevator;
        intakeSubsystem = intake;
        swerveSubsystem = swerve;
    }
    public void pitRoutine() {
        if (kElevatorEnabled) {
            elevatorCommands();
        }
        if (kIntakeEnabled) {
            intakeCommands();
        }
        if (kSwerveEnabled) {
            swerveCommands();
        }
    }
    public void intakeCommands() {
        Command zeroElevator = new ZeroElevator(elevatorSubsystem);
        zeroElevator.initialize();
        Command setElevatorHeight = new SetElevatorHeight(elevatorSubsystem, 1);
        setElevatorHeight.initialize();
    }
    public void elevatorCommands() {
        Command intakeCone = new IntakeCone(intakeSubsystem);
        intakeCone.initialize();
        Command intakeCube = new IntakeCube(intakeSubsystem);
        intakeCube.initialize();
    }
    public void swerveCommands() {
        LockSwerve lockSwerve = new LockSwerve(swerveSubsystem);
        lockSwerve.initialize();
        TeleopSwerve teleopSwerve = new TeleopSwerve(swerveSubsystem,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> driver.getRightX(),
                kFieldRelative,
                kOpenLoop);
        teleopSwerve.initialize();
        TeleopSwerveLimited teleopSwerveLimited = new TeleopSwerveLimited(swerveSubsystem,
                () -> driver.getRightY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                kFieldRelative,
                kOpenLoop);
        teleopSwerveLimited.initialize();
        TeleopSwerveWithAzimuth teleopSwerveWithAzimuth = new TeleopSwerveWithAzimuth(swerveSubsystem,
                () -> driver.getRightY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                kFieldRelative,
                kOpenLoop);
        teleopSwerveWithAzimuth.initialize();
    }
}
