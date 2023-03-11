// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

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
  public static void init() {

    // add preSink nodes
    ArrayList<PathNode> preSinks = preSink(blueDynamicPathWayNodes);
    PathNode topPreSink = preSinks.get(preSinks.size() - 1);
    PathNode botPreSink = preSinks.get(0);

    // add passages
    PathNode topPassageSink = new PathNode(2.8, 5.53 - 0.75, PathNode.NodeType.PASSAGE);
    PathNode topPassageSrc = new PathNode(5.81, 5.53 - 0.75, PathNode.NodeType.PASSAGE);
    ArrayList<PathNode> topPassage =
        passage(blueDynamicPathWayNodes, topPassageSrc, topPassageSink);
    // link top passage with top 2 PreSink
    PathUtil.fullyConnect(topPreSink, topPassageSink);
    PathUtil.fullyConnect(preSinks.get(preSinks.size() - 2), topPassageSink);

    PathNode botPassageSink = new PathNode(2.8, 0 + 0.73, PathNode.NodeType.PASSAGE);
    PathNode botPassageSrc = new PathNode(5.81, 0 + 0.73, PathNode.NodeType.PASSAGE);
    ArrayList<PathNode> botPassage =
        passage(blueDynamicPathWayNodes, botPassageSrc, botPassageSink);
    // link bottom passage with bottom 2 preSink
    PathUtil.fullyConnect(botPreSink, botPassageSink);
    PathUtil.fullyConnect(preSinks.get(1), botPassageSink);

    // add station nodes
    PathNode leftStation = new PathNode(6.28, 6.39, PathNode.NodeType.NORMAL);
    blueDynamicPathWayNodes.add(leftStation);
    // link left station node with top and bot passage src
    PathUtil.fullyConnect(leftStation, topPassageSrc);
    PathUtil.fullyConnect(leftStation, botPassageSrc);

    PathNode rightStation =
        new PathNode(Constants.FieldConstants.kFieldLength - 6.28, 6.39, PathNode.NodeType.NORMAL);
    blueDynamicPathWayNodes.add(rightStation);
    // link right station node with top and bot passage src
    PathUtil.fullyConnect(rightStation, topPassageSrc);
    PathUtil.fullyConnect(rightStation, botPassageSrc);

    // create redDynamicPathWayNodes
    for (PathNode node : blueDynamicPathWayNodes) {
      redDynamicPathWayNodes.add(node.getRedVersion());
    }
    System.out.println("Dynamic Path Way Nodes created");

    // display dynamic path way nodes in Path Planner
    displayWayNodes(blueDynamicPathWayNodes, true);
    displayWayNodes(redDynamicPathWayNodes, false);
  }

  public static void displayWayNodes(ArrayList<PathNode> nodes, boolean blue) {
    Path path = new Path(blueDynamicPathWayNodes, new Rotation2d(0), new Rotation2d(0));
    JSONObject json = path.getJson();
    String fileName = (blue ? "Blue" : "Red") + "DynamicPathWayNodes";
    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    FileUtil.saveJson(json, pathPlannerJsonPath);
  }

  public static ArrayList<PathNode> preSink(ArrayList<PathNode> pathNodes) {
    ArrayList<PathNode> preSinks = new ArrayList<>();
    for (Translation2d sink : Constants.FieldConstants.Grids.kLowTranslations) {
      preSinks.add(new PathNode(preSinkX, sink.getY(), PathNode.NodeType.PRESINK));
    }
    // shift the ends of the preSink inwards to avoid colliding into the wall
    preSinks.get(0).addY(preSinkEndpointsOffset);
    preSinks.get(preSinks.size() - 1).addY(-preSinkEndpointsOffset);
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
