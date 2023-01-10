package frc.robot.commands;

import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.Constants.IDConstants.*;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;
import org.apache.commons.math3.analysis.function.Power;

import java.util.logging.Logger;

public class PDHFaultWatcher extends CommandBase {
    private static final RobotLogger logger = new RobotLogger(PDHFaultWatcher.class.getCanonicalName());

    PowerDistribution powerDistribution = new PowerDistribution(PDH_ID, PowerDistribution.ModuleType.kRev);
    Timer timer = new Timer();

    boolean isBrownoutLogged = false;
    boolean isCanBusOffLogged = false;
    boolean isCanWarningLogged = false;

    boolean isChannel0BreakerFaultLogged = false;
    boolean isChannel1BreakerFaultLogged = false;
    boolean isChannel2BreakerFaultLogged = false;
    boolean isChannel3BreakerFaultLogged = false;
    boolean isChannel4BreakerFaultLogged = false;
    boolean isChannel5BreakerFaultLogged = false;
    boolean isChannel6BreakerFaultLogged = false;
    boolean isChannel7BreakerFaultLogged = false;
    boolean isChannel8BreakerFaultLogged = false;
    boolean isChannel9BreakerFaultLogged = false;
    boolean isChannel10BreakerFaultLogged = false;
    boolean isChannel11BreakerFaultLogged = false;
    boolean isChannel12BreakerFaultLogged = false;
    boolean isChannel13BreakerFaultLogged = false;
    boolean isChannel14BreakerFaultLogged = false;
    boolean isChannel15BreakerFaultLogged = false;
    boolean isChannel16BreakerFaultLogged = false;
    boolean isChannel17BreakerFaultLogged = false;
    boolean isChannel18BreakerFaultLogged = false;
    boolean isChannel19BreakerFaultLogged = false;
    boolean isChannel20BreakerFaultLogged = false;
    boolean isChannel21BreakerFaultLogged = false;
    boolean isChannel22BreakerFaultLogged = false;
    boolean isChannel23BreakerFaultLogged = false;

    @Override
    public void initialize() {
        timer.start();
    }

    public void checkFaults(PowerDistributionStickyFaults stickyFaults){
        if (stickyFaults.Brownout && !isBrownoutLogged ) {
            logger.severe("Brownout Detected!");
            isBrownoutLogged = true;
        }
        if (stickyFaults.CanBusOff && !isCanBusOffLogged ) {
            logger.severe("Can Bus Off!");
            isCanBusOffLogged = true;
        }
        if (stickyFaults.CanWarning && !isCanWarningLogged ) {
            logger.severe("Can Warning Detected!");
            isCanWarningLogged = true;
        }
        if (stickyFaults.Channel0BreakerFault && !isChannel0BreakerFaultLogged ) {
            logger.severe("Channel 0 Breaker Fault!");
            isChannel0BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel1BreakerFault && !isChannel1BreakerFaultLogged ) {
            logger.severe("Channel 1 Breaker Fault!");
            isChannel1BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel2BreakerFault && !isChannel2BreakerFaultLogged ) {
            logger.severe("Channel 2 Breaker Fault!");
            isChannel2BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel3BreakerFault && !isChannel3BreakerFaultLogged ) {
            logger.severe("Channel 3 Breaker Fault!");
            isChannel3BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel4BreakerFault && !isChannel4BreakerFaultLogged ) {
            logger.severe("Channel 4 Breaker Fault!");
            isChannel4BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel5BreakerFault && !isChannel5BreakerFaultLogged ) {
            logger.severe("Channel 5 Breaker Fault!");
            isChannel5BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel6BreakerFault && !isChannel6BreakerFaultLogged ) {
            logger.severe("Channel 6 Breaker Fault!");
            isChannel6BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel7BreakerFault && !isChannel7BreakerFaultLogged ) {
            logger.severe("Channel 7 Breaker Fault!");
            isChannel7BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel8BreakerFault && !isChannel8BreakerFaultLogged ) {
            logger.severe("Channel 8 Breaker Fault!");
            isChannel0BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel9BreakerFault && !isChannel9BreakerFaultLogged ) {
            logger.severe("Channel 9 Breaker Fault!");
            isChannel9BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel10BreakerFault && !isChannel10BreakerFaultLogged ) {
            logger.severe("Channel 10 Breaker Fault!");
            isChannel10BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel11BreakerFault && !isChannel11BreakerFaultLogged ) {
            logger.severe("Channel 11 Breaker Fault!");
            isChannel11BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel12BreakerFault && !isChannel12BreakerFaultLogged ) {
            logger.severe("Channel 12 Breaker Fault!");
            isChannel12BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel13BreakerFault && !isChannel13BreakerFaultLogged ) {
            logger.severe("Channel 13 Breaker Fault!");
            isChannel13BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel14BreakerFault && !isChannel14BreakerFaultLogged ) {
            logger.severe("Channel 14 Breaker Fault!");
            isChannel14BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel15BreakerFault && !isChannel15BreakerFaultLogged ) {
            logger.severe("Channel 15 Breaker Fault!");
            isChannel15BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel16BreakerFault && !isChannel16BreakerFaultLogged ) {
            logger.severe("Channel 16 Breaker Fault!");
            isChannel16BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel17BreakerFault && !isChannel17BreakerFaultLogged ) {
            logger.severe("Channel 17 Breaker Fault!");
            isChannel17BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel18BreakerFault && !isChannel18BreakerFaultLogged ) {
            logger.severe("Channel 18 Breaker Fault!");
            isChannel18BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel19BreakerFault && !isChannel19BreakerFaultLogged ) {
            logger.severe("Channel 19 Breaker Fault!");
            isChannel19BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel20BreakerFault && !isChannel20BreakerFaultLogged ) {
            logger.severe("Channel 20 Breaker Fault!");
            isChannel20BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel21BreakerFault && !isChannel21BreakerFaultLogged ) {
            logger.severe("Channel 21 Breaker Fault!");
            isChannel21BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel22BreakerFault && !isChannel22BreakerFaultLogged ) {
            logger.severe("Channel 22 Breaker Fault!");
            isChannel22BreakerFaultLogged = true;
        }
        if (stickyFaults.Channel23BreakerFault && !isChannel23BreakerFaultLogged ) {
            logger.severe("Channel 23 Breaker Fault!");
            isChannel23BreakerFaultLogged = true;
        }

    }
    @Override
    public void execute() {
        PowerDistributionStickyFaults stickyFaults = powerDistribution.getStickyFaults();
        if(timer.advanceIfElapsed(PDH_FAULT_WATCHER_INTERVAL)){
            long start = System.currentTimeMillis();
            checkFaults(stickyFaults);
            long end = System.currentTimeMillis();
            long elapsedTime = end - start;
            System.out.println("elapsedTime" + elapsedTime);
        }

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
