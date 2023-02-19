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
  static double passageRes = 0.6;

  public static void init() {
    System.out.println("Path Generator Initialized");

    // add preSink nodes
    ArrayList<PathNode> preSinks = preSink(dynamicPathWayNodes);
    PathNode topPreSink = preSinks.get(preSinks.size() - 2);
    PathNode botPreSink = preSinks.get(1);

    // add passages
    PathNode topPassageSink = new PathNode(2.8, 5.53 - 0.75, true);
    PathNode topPassageSrc = new PathNode(5.81, 5.53 - 0.75, true);
    ArrayList<PathNode> topPassage = passage(dynamicPathWayNodes, topPassageSrc, topPassageSink);
    PathUtil.fullyConnect(topPreSink, topPassageSink);

    PathNode botPassageSink = new PathNode(2.8, 0 + 0.73, true);
    PathNode botPassageSrc = new PathNode(5.81, 0 + 0.73, true);
    ArrayList<PathNode> botPassage = passage(dynamicPathWayNodes, botPassageSrc, botPassageSink);
    PathUtil.fullyConnect(botPreSink, botPassageSink);

    // add station nodes
    PathNode leftStation = new PathNode(6.28, 6.39);
    dynamicPathWayNodes.add(leftStation);
    PathUtil.fullyConnect(leftStation, topPassageSrc);

    PathNode rightStation = new PathNode(16.5 - 6.28, 6.39);
    dynamicPathWayNodes.add(rightStation);
    PathUtil.fullyConnect(rightStation, topPassageSrc);

    // mirror all points if red
    if (!blue) {
      for (PathNode p : dynamicPathWayNodes) {
        p.setPoint(new Translation2d(16.5 - p.getX(), p.getY()));
      }
    }

    // display special points in Path Planner
    Path path = new Path(dynamicPathWayNodes, new Rotation2d(0), new Rotation2d(0));
    JSONObject json = path.getJson();
    String fileName = "SpecialPoints";
    String pathPlannerJsonPath = "src/main/deploy/pathplanner/" + fileName + ".path";
    FileUtil.saveJson(json, pathPlannerJsonPath);
  }

  public static ArrayList<PathNode> preSink(ArrayList<PathNode> pathNodes) {
    ArrayList<PathNode> preSinks = new ArrayList<>();
    for (Translation2d sink : Constants.FieldConstants.Grids.kLowTranslations) {
      preSinks.add(new PathNode(sink.getX() + 1, sink.getY()));
    }
    // shift the ends of the preSink inwards to avoid colliding into the wall
    preSinks.get(0).addY(0.25);
    preSinks.get(preSinks.size() - 2).addY(-0.25);
    pathNodes.addAll(preSinks);
    PathUtil.fullyConnect(preSinks);
    return preSinks;
  }

  public static ArrayList<PathNode> passage(
      ArrayList<PathNode> pathNodes, PathNode src, PathNode sink) {
    ArrayList<PathNode> newNodes = new ArrayList<>();
    newNodes.add(sink);
    for (double x = sink.getX() + passageRes; x <= src.getX() + passageRes; x += passageRes) {
      newNodes.add(new PathNode(x, sink.getY(), true));
    }
    newNodes.add(src);
    PathUtil.fullyConnect(newNodes);
    pathNodes.addAll(newNodes);
    return newNodes;
  }
}
