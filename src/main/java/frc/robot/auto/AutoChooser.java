package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TransferSubsystem;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static TrajectoryFactory trajectoryFactory;
    private static ShooterSubsystem flywheelSubsystem;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive, IntakeSubsystem intake, ShooterSubsystem flywheel, TransferSubsystem transfer) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
        flywheelSubsystem = flywheel;
        Paths.initialize(drive, intake, flywheel, transfer);

        autoChooser = new SendableChooser<>();

        if (drive != null) {
            Command doNothing = new DefaultDriveCommandRobotOriented(drive);
            autoChooser.setDefaultOption("Do Nothing", doNothing);

            Command zeroBallTaxi = Paths.get0BallTaxi();
            autoChooser.addOption("0 Ball Taxi | Start Tarmac | Any Side", zeroBallTaxi);

            // path planner dot is the shooter
            if (flywheel != null) {
                Command oneBallTaxi = Paths.get1BallTaxi();
                autoChooser.addOption("1 Ball Taxi | Start Tarmac | Any Side", oneBallTaxi);

                if (intake != null && transfer != null) {

                    Command oneBallPicked = Paths.getPleasePickUsOneBall();
                    autoChooser.addOption("JUST ONE BALL", oneBallPicked);

                    Command oneBallPickedHanger = Paths.getPleasePickUsHanger();
                    autoChooser.addOption("HANGER DEFENSE", oneBallPickedHanger);

                    Command oneBallPickedFender = Paths.getPleasePickUsFender();
                    autoChooser.addOption("FENDER DEFENSE", oneBallPickedFender);

                    Command oneBallTarmacFar1BallSide = Paths.get1BallOuttakeFarTarmac1BallSide();
                    autoChooser.addOption("1 Ball Outtake | Start Far Tarmac | 1 Ball Side", oneBallTarmacFar1BallSide);

                    Command cool = Paths.getCoolAuto();
                    autoChooser.addOption("COOL AUTO RUN THIS", cool);

                    Command twoBallTarmacMid2BallSide = Paths.get2BallMidTarmac2BallSide();
                    autoChooser.addOption("2 Ball Knock Red | Start Mid Tarmac | 2 Ball Side", twoBallTarmacMid2BallSide);

                    Command twoBallTarmacMid1BallSide = Paths.get2BallMidTarmac1BallSide();
                    autoChooser.addOption("2 Ball | Start Mid Tarmac | 1 Ball Side", twoBallTarmacMid1BallSide);

                    Command twoBallDefenseMidTarmac1BallSide = Paths.get2BallDefenseMidTarmac1BallSide();
                    autoChooser.addOption("2 Ball Outtake | Start Mid Tarmac | 1 Ball Side", twoBallDefenseMidTarmac1BallSide);

                    Command twoBallTarmacEdge2BallSide = Paths.get2BallFarTarmac2BallSide();
                    autoChooser.addOption("2 Ball | Start Edge Tarmac | 2 Ball Side", twoBallTarmacEdge2BallSide);

                    Command threeBallTarmacEdge2BallSide = Paths.get3BallFarTarmac2BallSide();
                    autoChooser.addOption("3 Ball | Start Edge Tarmac | 2 Ball Side", threeBallTarmacEdge2BallSide);

                    Command fourBallTarmacEdge2BallSide = Paths.get4BallFarTarmac2BallSide();
                    autoChooser.addOption("4/5 Ball | Start Edge Tarmac | 2 Ball Side", fourBallTarmacEdge2BallSide);

                    Command fourBallTarmacMid2BallSide = Paths.get4BallMidTarmac2BallSide();
                    autoChooser.addOption("4 Ball | Start Mid Tarmac | 2 Ball Side", fourBallTarmacMid2BallSide);
                }
            }
        }

        return autoChooser;
    }

    public static Command getCommand() {
        return flywheelSubsystem != null ?
                new ParallelRaceGroup(
                    new WaitCommand(3),
                    new ZeroHoodMotorCommand(flywheelSubsystem)
                ).andThen(autoChooser.getSelected())
                :
            autoChooser.getSelected();
    }
}
