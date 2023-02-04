package frc.robot.limelight;
//package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.helper.logging.RobotLogger;
//import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
//import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import static frc.robot.Constants.LimelightConstants.LIMELIGHT_TUNED_DATA;
import static frc.robot.Constants.LimelightConstants.*;

import static frc.robot.Constants.TARGET_HEIGHT_INCHES;
import static frc.robot.Constants.MOUNTING_HEIGHT_INCHES;
// TODO: create MOUNTING_HEIGHT_INCHES to import (create in constants)

public class Limelight {
    private static final RobotLogger logger = new RobotLogger(Limelight.class.getCanonicalName());
    private static NetworkTable limelight;
    private static PolynomialSplineFunction tunedDistance;
    private static int users = 0;

    static {
//        double[] rawDistance = new double[LIMELIGHT_TUNED_DATA.size()];
//        double[] actualDistance = new double[LIMELIGHT_TUNED_DATA.size()];
//        for(int i = 0; i < LIMELIGHT_TUNED_DATA.size(); i++) {
//            int[] data = LIMELIGHT_TUNED_DATA.get(i);
//            rawDistance[i] = data[0];
//            actualDistance[i] = data[1];
//        }
//
//        tunedDistance = new SplineInterpolator().interpolate(rawDistance, actualDistance);
    }

    // Doesn't Allow Instancing
    private Limelight(){}

    public static void init() {
        // Setting up NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        limelight = inst.getTable("limelight");

        // Setting up default stream
        limelight.getEntry("ledMode").setNumber(1); // Forces the LED Mode to off
        limelight.getEntry("camMode").setNumber(0); // Uses Vision Processor Mode
        limelight.getEntry("pipeline").setNumber(0); // Uses pipeline #0
        limelight.getEntry("stream").setNumber(2); // Driver Camera Main, Vision Camera Lower-Right Corner
        limelight.getEntry("snapshot").setNumber(0); // Takes no snapshots

        if(getLimelightValue("tx").getDouble(1000) == 1000)
            logger.warning("Limelight Not Responding");
    }
    /**
     * @param value
     * @return entry with name of value
     */
    private static NetworkTableEntry getLimelightValue(String value){
        if (limelight == null) {
            logger.severe("Limelight not Initialized! Returning Bad NetworkTable!");
            return new NetworkTableEntry(NetworkTableInstance.getDefault(), 0);
        }
        return limelight.getEntry(value);
    }
    /**
     * @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public static double getTx(){
        return getLimelightValue("tx").getDouble(0);
    }
    /**
     * @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    public static double getTy(){
        return getLimelightValue("ty").getDouble(0);
    }
    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public static double getTa(){
        return getLimelightValue("ta").getDouble(0);
    }
    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public static double getTs(){
        return getLimelightValue("ts").getDouble(0);
    }
    /**
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public static double getTv(){
        return getLimelightValue("tv").getDouble(0);
    }
    /**
     * @return Number array of corner coordinates [x0,x1,etc]
     */
    public static double[] getTcornx(){
        return getLimelightValue("tcornx").getDoubleArray(new double[4]);
    }
    /**
     * @return Number array of corner coordinates [y0,y1,etc]
     */
    public static double[] getTcorny(){
        return getLimelightValue("tcorny").getDoubleArray(new double[4]);
    }
    /**
     * @return raw distance to target (inches)
     */
    public static double getRawDistanceToTarget(){
        return (TARGET_HEIGHT_INCHES-MOUNTING_HEIGHT_INCHES)/Math.tan(toRadians(MOUNTING_ANGLE_DEG+getTy()));
    }

    /**
     * @return tuned distance to target (inches)
     */
    public static double getTunedDistanceToTarget(){
        double rawDistance = getRawDistanceToTarget();
        return rawDistance;
        try {
            return tunedDistance.value(rawDistance);
        } catch (Exception e) {
            logger.warning("Distance from Limelight is out of range of interpolating");
            return rawDistance;
        }
    }

    public static boolean isTargetDetected(){
        if(getTv() == 1){
            return true;
        }
        else{
            return false;
        }
    }

    /**
     * @param degrees
     * @return radians
     */
    public static double toRadians(double degrees){
        return degrees * Math.PI/180.0;
    }


    /**
     * Enables the LEDs on the limelight, LEDs should be on when limelight is in use.
     */
    public static void enable(){
        users++;
        Limelight.getLimelightValue("ledMode").setNumber(3);
    }

    public static void disable(){
        if (--users == 0) {
            Limelight.getLimelightValue("ledMode").setNumber(1);
        }
    }


}