// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathFinder.doesPathHitObstacles;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.dynamicPathAllowedPositions;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathGenerationConstants.dynamicPathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerve.SwerveConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.PriorityQueue;

public class HeuristicHelper {
	static double ILLEGAL_PENALTY = Double.MAX_VALUE/20;
	static double INF = Double.MAX_VALUE/10;
  public static double[] generateHeuristic(int sink, ArrayList<Translation2d> positions) {
    // fill in heurestic
    int nodes = positions.size();
    double[] heurestic = new double[nodes];
    boolean[] vis = new boolean[nodes];
    Arrays.fill(heurestic, INF);
    heurestic[sink] = 0;
    PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> heurestic[a]));
    while (!pq.isEmpty()) {
      int next = pq.poll();
      vis[next] = true;
      for (int node = 0; node < nodes; node++) {
        if (vis[node]) continue;
        double newHeurestic = heuristic(
            positions.get(node), positions.get(next));
        if (heurestic[node] < newHeurestic) continue;
        heurestic[node] = newHeurestic;
        pq.add(node);
      }
    }
    for (int i = 0; i < nodes; i++) {
      heurestic[i] = heurestic[i] / dynamicPathConstraints.maxVelocity;
    }
    return heurestic;
  }

	// heuristic estimate of time to travel 1->2 that is guaranteed to be lower than
	// actual
	public static double heuristic(Translation2d position1, Translation2d position2) {
		double ret = position1.getDistance(position2) / SwerveConstants.kMaxSpeed;
		if (doesPathHitObstacles(position1, position2)) {
			ret += ILLEGAL_PENALTY;
		}
		return ret;
	}
}
