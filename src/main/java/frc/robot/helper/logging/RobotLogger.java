package frc.robot.helper.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.PrintWriter;
import java.io.StringWriter;

import static frc.robot.Constants.DEBUG;
import static frc.robot.Constants.LOG_DEBUG_TO_CONSOLE;

public class RobotLogger {

    private final String className;

    // Shared between all Robot Logger Instances
    private static final DataLog log = DataLogManager.getLog();
    private static StringLogEntry debug;
    private static StringLogEntry info;
    private static StringLogEntry warning;
    private static StringLogEntry severe;

    /**
     * Initializes local logger for individual class
     * @param className Name of class, usually can get via {@code MyClass.class.getCanonicalName()}
     */
    public RobotLogger(String className){
        this.className = className;

        // If Already Initialized, Don't Reinitialize
        if (info != null)
            return;

        if (DEBUG)
            debug = new StringLogEntry(log, "messages/debug");

        info = new StringLogEntry(log, "messages/info");
        warning = new StringLogEntry(log, "messages/warning");
        severe = new StringLogEntry(log, "messages/severe");
    }

    /**
     * Initializes WPILib Logger, should be placed in robotInit()
     */
    public static void init(){
        DataLogManager.start();
        DataLogManager.logNetworkTables(false);
    }

    /**
     * Message that only gets logged to file with DEBUG enabled.
     * Separate Constant to set whether it gets logged to DriverStation.
     *
     * @param message Message to be logged
     */
    public void debug(String message){
        if (DEBUG) {
            debug.append(getClassName() + message);
            if (LOG_DEBUG_TO_CONSOLE){
                System.out.println(getClassName() + message);
            }
        }
    }

    /**
     * Helper method that logs an object's toString.
     *
     * Message only gets logged to file with DEBUG enabled.
     * Separate Constant to set whether it gets logged to DriverStation.
     *
     * @param objName Name of Object
     * @param obj Object that toString() will be called on
     */
    public void debug(String objName, Object obj) {
        debug(getClassName() + objName + ": " + obj);
    }

    /**
     * Always gets logged to file. Won't be printed to DriverStation unless DEBUG is enabled.
     *
     * @param message Message to log
     */
    public void info(String message){
        info.append(getClassName() + message);

        System.out.println(getClassName() + message);
    }

    /**
     * Always Gets Printed to both DriverStation and Log File
     *
     * @param message Message to log as warning
     */
    public void warning(String message){
        warning.append(getClassName() + message);
        DriverStation.reportWarning(getClassName() + message, false);
    }

    /**
     * Always Gets Printed to both DriverStation and Log File, also prints stacktrace on newline
     *
     * @param message Message to log as warning
     * @param throwable Throwable Exception to Print Stacktrace
     */
    public void warning(String message, Throwable throwable) {
        StringWriter finalMsg = new StringWriter();

        finalMsg.append(getClassName() + message);
        finalMsg.append("\n");

        // Stack Trace Requires Print Writer
        PrintWriter printWriter = new PrintWriter(finalMsg);
        throwable.printStackTrace(printWriter);


        warning.append(finalMsg.toString());
        DriverStation.reportWarning(getClassName() + message, throwable.getStackTrace());
    }

    /**
     * Always Gets Printed to both DriverStation and Log File
     *
     * @param message Message to log as severe
     */
    public void severe(String message){
        severe.append(getClassName() + message);
        DriverStation.reportError(getClassName() + message, true);
    }

    /**Always Gets Printed to both DriverStation and Log File
     *
     * @param message Message to log as severe
     * @param throwable Throwable Exception to Print Stacktrace
     */
    public void severe(String message, Throwable throwable){
        StringWriter finalMsg = new StringWriter();

        finalMsg.append(getClassName() + message);
        finalMsg.append("\n");

        // Stack Trace Requires Print Writer
        PrintWriter printWriter = new PrintWriter(finalMsg);
        throwable.printStackTrace(printWriter);

        severe.append(finalMsg.toString());
    }

    private String getClassName(){
        return "[ " + className + " ] ";
    }
}