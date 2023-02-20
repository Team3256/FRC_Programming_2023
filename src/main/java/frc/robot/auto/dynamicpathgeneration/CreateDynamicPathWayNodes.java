// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.blue;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.dynamicPathWayNodes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.auto.dynamicpathgeneration.helpers.FileUtil;
import frc.robot.auto.dynamicpathgeneration.helpers.Path;
import frc.robot.auto.dynamicpathgeneration.helpers.PathNode;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import java.util.ArrayList;
import org.json.simple.JSONObject;

public class CreateDynamicPathWayNodes {
  static double passagePoints = 8;

  public static void init() {
    System.out.println("Path Generator Initialized");

    // add preSink nodes
    ArrayList<PathNode> preSinks = preSink(dynamicPathWayNodes);
    PathNode topPreSink = preSinks.get(preSinks.size() - 1);
    PathNode botPreSink = preSinks.get(0);

    // add passages
    PathNode topPassageSink = new PathNode(2.8, 5.53 - 0.75, PathNode.NodeType.PASSAGE);
    PathNode topPassageSrc = new PathNode(5.81, 5.53 - 0.75, PathNode.NodeType.PASSAGE);
    ArrayList<PathNode> topPassage = passage(dynamicPathWayNodes, topPassageSrc, topPassageSink);
    // link top passage with top PreSink
    PathUtil.fullyConnect(topPreSink, topPassageSink);

    PathNode botPassageSink = new PathNode(2.8, 0 + 0.73, PathNode.NodeType.PASSAGE);
    PathNode botPassageSrc = new PathNode(5.81, 0 + 0.73, PathNode.NodeType.PASSAGE);
    ArrayList<PathNode> botPassage = passage(dynamicPathWayNodes, botPassageSrc, botPassageSink);
    // link bottom passage with bottom preSink
    PathUtil.fullyConnect(botPreSink, botPassageSink);

    // add station nodes
    PathNode leftStation = new PathNode(6.28, 6.39);
    dynamicPathWayNodes.add(leftStation);
    // link left station node with top passage src
    PathUtil.fullyConnect(leftStation, topPassageSrc);

    PathNode rightStation = new PathNode(Constants.FieldConstants.kFieldLength - 6.28, 6.39);
    dynamicPathWayNodes.add(rightStation);
    // link right station node with top passage src
    PathUtil.fullyConnect(rightStation, topPassageSrc);

    // mirror all dynamic path way nodes if red
    if (!blue) {
      for (PathNode p : dynamicPathWayNodes) {
        p.setPoint(new Translation2d(Constants.FieldConstants.kFieldLength - p.getX(), p.getY()));
      }
    }

    // display dynamic path way nodes in Path Planner
    Path path = new Path(dynamicPathWayNodes, new Rotation2d(0), new Rotation2d(0));
    JSONObject json = path.getJson();
    String fileName = "DynamicPathWayNodes";
    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    FileUtil.saveJson(json, pathPlannerJsonPath);
  }

  public static ArrayList<PathNode> preSink(ArrayList<PathNode> pathNodes) {
    double preSinkX = 2.1;
    ArrayList<PathNode> preSinks = new ArrayList<>();
    for (Translation2d sink : Constants.FieldConstants.Grids.kLowTranslations) {
      preSinks.add(new PathNode(preSinkX, sink.getY(), PathNode.NodeType.PRESINK));
    }
    // shift the ends of the preSink inwards to avoid colliding into the wall
    preSinks.get(0).addY(0.30);
    preSinks.get(preSinks.size() - 1).addY(-0.30);
    pathNodes.addAll(preSinks);
    PathUtil.fullyConnect(preSinks);
    return preSinks;
  }

  public static ArrayList<PathNode> passage(
      ArrayList<PathNode> pathNodes, PathNode src, PathNode sink) {
    ArrayList<PathNode> newNodes = new ArrayList<>();
    newNodes.add(sink);
    double passageResolution = (src.getX() - sink.getX()) / (passagePoints - 1);
    for (int i = 1; i <= passagePoints - 2; i++) {
      newNodes.add(
          new PathNode(
              sink.getX() + i * passageResolution, sink.getY(), PathNode.NodeType.PASSAGE));
    }
    newNodes.add(src);
    PathUtil.fullyConnect(newNodes);
    pathNodes.addAll(newNodes);
    return newNodes;
  }
}
