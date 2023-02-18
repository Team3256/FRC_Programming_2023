// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.dynamicPathWayNodes;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kDynamicPathGenerationDebug;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.ArrayList;
import org.json.simple.JSONObject;

public class PathGenInit {
  static double passageRes = 0.6;
  // assume blue for now
  public static void init() {
    System.out.println("Path Generator Initialized");

    // end
    preSink(dynamicPathWayNodes);

    // passages
    PathNode topPassageSrc = new PathNode(2.28, 5.53 - 0.75);
    PathNode topPassageSink = new PathNode(5.81, 5.53 - 0.75);
    ArrayList<PathNode> topPassage = passage(dynamicPathWayNodes, topPassageSrc, topPassageSink);

    PathNode botPassageSrc = new PathNode(2.28, 0 + 0.73);
    PathNode botPassageSink = new PathNode(5.81, 0 + 0.73);
    ArrayList<PathNode> botPassage = passage(dynamicPathWayNodes, botPassageSrc, botPassageSink);

    // station
    PathNode leftStation = new PathNode(6.28, 6.39);
    dynamicPathWayNodes.add(leftStation);
    fullyConnect(leftStation, topPassage);

    PathNode rightStation = new PathNode(16.5 - 6.28, 6.39);
    dynamicPathWayNodes.add(rightStation);
    fullyConnect(rightStation, topPassageSink);

    // debug
    ArrayList<Translation2d> pathPositions = new ArrayList<>();
    for (PathNode pathNode : dynamicPathWayNodes) pathPositions.add(pathNode.getPoint());
    Path path = new Path(pathPositions, new Rotation2d(0), new Rotation2d(0));
    JSONObject json = path.getJson();
    String fileName = "SpecialPoints";
    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    FileHelper.saveJson(json, pathPlannerJsonPath);
  }

  public static void preSink(ArrayList<PathNode> pathNodes) {
    ArrayList<PathNode> preSinks = new ArrayList<>();
    for (Translation2d sink : Constants.FieldConstants.Grids.kLowTranslations) {
      preSinks.add(new PathNode(sink.getX() + 1, sink.getY()));
    }
    pathNodes.addAll(preSinks);
    fullyConnect(preSinks);
    if (kDynamicPathGenerationDebug) {
      System.out.println("preSink nodes:" + preSinks.size());
      System.out.println("preSink edges:" + preSinks.get(0).getEdges().size());
    }
  }

  public static ArrayList<PathNode> passage(
      ArrayList<PathNode> pathNodes, PathNode src, PathNode sink) {
    ArrayList<PathNode> newNodes = new ArrayList<>();
    newNodes.add(src);
    for (double x = src.getX() + passageRes; x <= sink.getX() - passageRes; x += passageRes) {
      newNodes.add(new PathNode(x, src.getY()));
    }
    newNodes.add(sink);
    fullyConnect(newNodes);
    if (kDynamicPathGenerationDebug) {
      System.out.println("passage nodes:" + newNodes.size());
      System.out.println("passage edges:" + newNodes.get(0).getEdges().size());
    }
    pathNodes.addAll(newNodes);
    return newNodes;
  }

  public static void fullyConnect(ArrayList<PathNode> pathNodes) {
    for (PathNode u : pathNodes) {
      for (PathNode v : pathNodes) {
        if (u != v) u.addEdge(v);
      }
    }
  }

  public static void fullyConnect(ArrayList<PathNode> pathNodes1, ArrayList<PathNode> pathNodes2) {
    for (PathNode u : pathNodes1) {
      for (PathNode v : pathNodes2) {
        fullyConnect(u, v);
      }
    }
  }

  public static void fullyConnect(PathNode u, ArrayList<PathNode> pathNodes) {
    for (PathNode v : pathNodes) {
      fullyConnect(u, v);
    }
  }

  public static void fullyConnect(PathNode u, PathNode v) {
    u.addEdge(v);
    v.addEdge(u);
  }
}
