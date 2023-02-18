// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

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
    ArrayList<PathNode> pathNodes = new ArrayList<>();

    // end
    PathNode[] res = preSink(pathNodes);
    PathNode topPreSink = res[0];
    PathNode botPreSink = res[1];

    // passages
    PathNode topPassageSrc = new PathNode(2.28, 5.53 - 0.75);
    PathNode topPassageSink = new PathNode(5.81, 5.53 - 0.75);
    ArrayList<PathNode> topPassage = passage(pathNodes, topPassageSrc, topPassageSink);

    PathNode botPassageSrc = new PathNode(2.28, 0 + 0.73);
    PathNode botPassageSink = new PathNode(5.81, 0 + 0.73);
    ArrayList<PathNode> botPassage = passage(pathNodes, botPassageSrc, botPassageSink);

    // station
    PathNode leftStation = new PathNode(6.28, 6.39);
    pathNodes.add(leftStation);
    fullyConnect(leftStation, topPassage);

    PathNode rightStation = new PathNode(16.5 - 6.28, 6.39);
    pathNodes.add(rightStation);
    fullyConnect(rightStation, topPassageSink);

    // debug
    ArrayList<Translation2d> pathPositions = new ArrayList<>();
    for (PathNode pathNode : pathNodes) pathPositions.add(pathNode.getPoint());
    Path path = new Path(pathPositions, new Rotation2d(0), new Rotation2d(0));
    JSONObject json = path.getJson();
    String fileName = "SpecialPoints";
    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    FileHelper.saveJson(json, pathPlannerJsonPath);
  }

  static PathNode[] preSink(ArrayList<PathNode> pathNodes) {
    ArrayList<PathNode> preSinks = new ArrayList<>();
    for (Translation2d sink : Constants.FieldConstants.Grids.kLowTranslations) {
      preSinks.add(new PathNode(sink.getX() + 1, sink.getY()));
    }
    pathNodes.addAll(preSinks);
    fullyConnect(preSinks);
    return new PathNode[] {preSinks.get(0), preSinks.get(preSinks.size() - 1)};
  }

  static ArrayList<PathNode> passage(ArrayList<PathNode> pathNodes, PathNode src, PathNode sink) {
    ArrayList<PathNode> newNodes = new ArrayList<>();
    newNodes.add(src);
    for (double x = src.getX() + passageRes; x <= sink.getX() - passageRes; x += passageRes) {
      newNodes.add(new PathNode(x, src.getY()));
    }
    newNodes.add(sink);
    fullyConnect(newNodes);
    pathNodes.addAll(newNodes);
    return newNodes;
  }

  static void fullyConnect(ArrayList<PathNode> pathNodes) {
    for (PathNode u : pathNodes) {
      for (PathNode v : pathNodes) {
        if (u != v) fullyConnect(u, v);
      }
    }
  }

  static void fullyConnect(ArrayList<PathNode> pathNodes1, ArrayList<PathNode> pathNodes2) {
    for (PathNode u : pathNodes1) {
      for (PathNode v : pathNodes2) {
        fullyConnect(u, v);
      }
    }
  }

  static void fullyConnect(PathNode u, ArrayList<PathNode> pathNodes) {
    for (PathNode v : pathNodes) {
      fullyConnect(u, v);
    }
  }

  static void fullyConnect(PathNode u, PathNode v) {
    u.addEdge(u);
    v.addEdge(u);
  }
}
