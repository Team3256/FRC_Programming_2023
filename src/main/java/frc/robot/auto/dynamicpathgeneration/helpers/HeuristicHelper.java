// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathFinder.doesPathSegmentHitObstacles;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerve.SwerveConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.PriorityQueue;

public class HeuristicHelper {
  public static double[] generateHeuristicTable(int sink, ArrayList<Translation2d> positions) {
    // fill in heurestic
    int nodes = positions.size();
    double[] heurestic = new double[nodes];
    boolean[] vis = new boolean[nodes];
    Arrays.fill(heurestic, INF_TIME);
    heurestic[sink] = 0;
    PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> heurestic[a]));
    pq.add(sink);
    while (!pq.isEmpty()) {
      int cur = pq.poll();
      if (vis[cur]) continue;
      vis[cur] = true;
      // System.out.println("cur:" + cur);
      for (int next = 0; next < nodes; next++) {
        if (vis[next]) continue;
        double newHeurestic =
            heurestic[cur] + splineHeuristic(positions.get(cur), positions.get(next));
        if (heurestic[next] < newHeurestic) continue;
        heurestic[next] = newHeurestic;
        pq.add(next);
      }
    }
    return heurestic;
  }

  // heavy splined heuristic estimate of time to travel 1->2 that is guaranteed to
  // be lower than
  // actual
  public static double splineHeuristic(Translation2d position1, Translation2d position2) {
    double estimatedTime = position1.getDistance(position2) / (SwerveConstants.kMaxSpeed);
    if (doesPathSegmentHitObstacles(position1, position2)) {
      estimatedTime += ILLEGAL_TIME;
    }
    return estimatedTime;
  }
}
