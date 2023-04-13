// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import java.util.List;

public class StatisticsHelper {
  public static double calculateStandardDeviation(List<Double> array) {
    double mean = calculateMean(array);

    double standardDeviation = 0.0;
    for (double num : array) {
      standardDeviation += Math.pow(num - mean, 2);
    }

    return Math.sqrt(standardDeviation / (array.size() - 1));
  }

  public static double calculateMean(List<Double> array) {
    double sum = 0.0;
    for (double i : array) {
      sum += i;
    }

    int length = array.size();
    return sum / length;
  }
}
