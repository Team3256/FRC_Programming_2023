// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.ArrayList;

public class PathNode {
  private static int curIndex = 0;
  private Translation2d point;
  private ArrayList<Integer> edges;
  private int index;
  private NodeType nodeType;

  public enum NodeType {
    NORMAL,
    PASSAGE,
    PRESINK,
    SRC,
    SINK
  }

  public PathNode(double x, double y, NodeType nodeType) {
    if (nodeType == NodeType.SRC) {
      index = curIndex;
    } else if (nodeType == NodeType.SINK) {
      index = curIndex + 1;
    } else {
      index = curIndex;
      curIndex++;
    }
    this.point = new Translation2d(x, y);
    this.nodeType = nodeType;
    edges = new ArrayList<>();
  }

  private PathNode(Translation2d point, NodeType nodeType, int index, ArrayList<Integer> edges) {
    this.point = point;
    this.nodeType = nodeType;
    this.index = index;
    this.edges = new ArrayList<>();
    this.edges.addAll(edges);
  }

  public PathNode getRedVersion() {
    Translation2d newPoint =
        new Translation2d(Constants.FieldConstants.kFieldLength - getX(), getY());
    return new PathNode(newPoint, nodeType, index, edges);
  }

  public void addEdge(Integer node) {
    edges.add(node);
  }

  public void remEdge(Integer node) {
    edges.remove(node);
  }

  public void addX(double x) {
    point = point.plus(new Translation2d(x, 0));
  }

  public void addY(double y) {
    point = point.plus(new Translation2d(0, y));
  }

  public double getX() {
    return point.getX();
  }

  public double getY() {
    return point.getY();
  }

  public int getIndex() {
    return index;
  }

  public NodeType getType() {
    return nodeType;
  }

  public Translation2d getPoint() {
    return point;
  }

  public void setPoint(Translation2d point) {
    this.point = point;
  }

  public ArrayList<Integer> getEdges() {
    return edges;
  }

  public String toString() {
    return "[" + index + ": " + getX() + ", " + getY() + "]";
  }
}
