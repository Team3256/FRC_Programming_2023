// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.drivers.CanDeviceId;
import frc.robot.mole.helper.TrainingDataPoint;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public final class MoleConstants {

  public static final int kMoleMotorID = 31;
  public static final int kMolePivotMotorID = 32;

  public static final String kMoleCANBus = "mani";
  public static final String kMolePivotCANBus = "mani";

  public static final CanDeviceId kMoleCANDevice = new CanDeviceId(kMoleMotorID, kMoleCANBus);
  public static final CanDeviceId kMolePivotCANDevice =
      new CanDeviceId(kMolePivotMotorID, kMolePivotCANBus);
  public static final double kMoleCurrentSpikingThreshold = 40;
  public static Rotation2d kDefaultMoleAngle = Rotation2d.fromDegrees(0);
  public static Rotation2d kCubeMidAngle = Rotation2d.fromDegrees(45);
  public static Rotation2d kCubeHighAngle = Rotation2d.fromDegrees(60);

  public static final double kDefaultSpeed = 0;
  public static final double kCubeMidSpeed = 0.5;
  public static final double kCubeHighSpeed = 0.9;

  // Pivot Motor Constants
  public static final double kMolePivotGearing = 60;
  public static final double kMolePivotInertia = 60;
  public static final double kMoleLength = 10;

  public static final Rotation2d kMinConstraintAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d kMaxConstraintAngle = Rotation2d.fromDegrees(90);

  public static final TrapezoidProfile.Constraints kProfileContraints =
      new TrapezoidProfile.Constraints(8, 4);

  public static Rotation2d kMolePivotAngleTolerance = Rotation2d.fromDegrees(1);

  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kArmS = 0;
  public static final double kArmG = 0;
  public static final double kArmV = 0;
  public static final double kArmA = 0;

  public static final List<TrainingDataPoint> MOLE_INTERP_DATA =
      Arrays.asList(
          // tuned 9/10
          new TrainingDataPoint(57.00706, 000000));

  public static final PolynomialSplineFunction distanceToMoleShooterRPMInterpolation;

  static {
    double[] trainDistance = new double[MOLE_INTERP_DATA.size()];
    double[] trainMoleShooterRPM = new double[MOLE_INTERP_DATA.size()];

    for (int point = 0; point < MOLE_INTERP_DATA.size(); ++point) {
      trainDistance[point] = MOLE_INTERP_DATA.get(point).distance;
      trainMoleShooterRPM[point] = MOLE_INTERP_DATA.get(point).moleShooterRPM;
    }

    distanceToMoleShooterRPMInterpolation =
        new LinearInterpolator().interpolate(trainDistance, trainMoleShooterRPM);
  }

  public static double kMoleInterpolationMinValue = 0.1;
  public static double kMoleInterpolationMaxValue = 1;
}
