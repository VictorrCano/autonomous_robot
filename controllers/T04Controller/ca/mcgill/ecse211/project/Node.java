package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.playingfield.Point;

public class Node {
  public Point currentPoint;
  public Node previous;
  
  /**
   * Node in list.
   * @param currentPoint previous node in list
   * @param previous next node in list
   */
  public Node(Point currentPoint, Node previous) {
    this.currentPoint = currentPoint;
    this.previous = previous;
  }
  
}
