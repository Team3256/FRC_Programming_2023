// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class PathNode {
  Translation2d point;
  ArrayList<PathNode> edges;
  int index;
  boolean isPassage;

  public PathNode(double x, double y) {
    this.point = new Translation2d(x, y);
    this.edges = new ArrayList<>();
  }

  public PathNode(double x, double y, boolean isPassage) {
    this(x, y);
    this.isPassage = isPassage;
  }

  public PathNode(Translation2d point) {
    this.point = point;
    this.edges = new ArrayList<>();
  }

  public void addEdge(PathNode node) {
    edges.add(node);
  }

  public void remEdge(PathNode node) {
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

  public void setIndex(int index) {
    this.index = index;
  }

  public boolean isPassage() {
    return isPassage;
  }

  public Translation2d getPoint() {
    return point;
  }

  public ArrayList<PathNode> getEdges() {
    return edges;
  }

  public String toString() {
    return "[" + index + ": " + getX() + ", " + getY() + "]";
  }
}
