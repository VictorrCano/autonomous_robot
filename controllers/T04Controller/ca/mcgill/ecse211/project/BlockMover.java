package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.LightLocalizer.moveTillLine;
import static ca.mcgill.ecse211.project.Resources.BOX_WIDTH;
import static ca.mcgill.ecse211.project.Resources.CORRECTION;
import static ca.mcgill.ecse211.project.Resources.DEGREE_TO_RAD;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.VECTORS_FILE;
import static ca.mcgill.ecse211.project.Resources.island;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.redTeam;
import static ca.mcgill.ecse211.project.Resources.torquemeter;

import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.RampEdge;
import ca.mcgill.ecse211.playingfield.Region;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Queue;



public class BlockMover {
  /** Boolean used to determine if robot is currently moving a block. */
  public static boolean isMoving = false;
  /** Array to store the averages of torque measured while pushing each block. */
  // public static double[] torqueAverages = new double [Main.vectors.size()];

  public static int correctionTravelTo = 0;

  /** Array to store the averages of torque measured while pushing each block. */
  public static double[] torqueAverages = new double[10];
  public static ArrayList<Point> possibleGoodPoints;
  public static Region searchZone = null;
  public static RampEdge ramp = null;

  public boolean isMoving() {
    return isMoving;
  }

  /**
   * Tells robot to move forward and push the block into the bin,
   * stopping when the motors think there is no longer a
   * block.
   */
  public static void pushBlockIntoBin() {
    Resources.leftMotor.forward();
    Resources.rightMotor.forward();

  }

  /**
   * Make the robot move the block, i, from p1 to p2.
   * 
   * @param i : Number of block to move
   * @param p1 : Point of initial pos of block
   * @param p2 : Point of final pos of block
   */
  public static void moveBlockTo(Integer i, Point p1, Point p2) {
    Double angle = Navigation.getDestinationAngle(p1, p2);
    Double oppositeAngle = (angle + 180) % 360;

    Point currentPosition = new Point(odometer.getXyt()[0] / TILE_SIZE,
        odometer.getXyt()[1] / TILE_SIZE);

    // get the destination values of X and Y
    double destinationX = (p1.x) + (Math.sin(oppositeAngle * DEGREE_TO_RAD));
    double destinationY = p1.y + Math.cos(oppositeAngle * DEGREE_TO_RAD);

    Point destination = new Point(destinationX, destinationY);
    int correction = 0;
    double deltaX = currentPosition.x - destination.x;
    double deltaY = currentPosition.y - destination.y;

    double deltaX1 = p1.x - p2.x;
    double deltaY1 = p1.y - p2.y;
    int length = (int) Math.sqrt((deltaX1 * deltaX1) + (deltaY1 * deltaY1));

    // internal method correction
    correction = length / 3;
    // correction in the travelTo Navigation method that is called
    correctionTravelTo = correction * 2;


    if (Math.sqrt((deltaX * deltaX) + (deltaY * deltaY)) < 2.0) {

      correction = 0;
      correctionTravelTo = 0;
    }



    Navigation.turnBy(angle - correction);
    Torquemeter.initialize();


    // get to the destination
    Navigation.travelTo(destination);
    Navigation.turnBy(angle);
    // perform an error correction
    Drive.turnBy(correction);


    // set up the torque meter to find heaviest block
    Torquemeter.initialize();
    new Thread(torquemeter).start();
    // if we are heading to the last point in map three
    if (p2.x == 6 && p2.y == 3) {
      // adjust the correction
      Drive.moveStraightFor(
          (Navigation.distanceBetween(p1, p2) * TILE_SIZE) 
          + TILE_SIZE - (BOX_WIDTH / 2) - CORRECTION / 2.0,
          correction * 4);
      // if we are using map 1 and on block 3
    } else if (VECTORS_FILE.getFileName().toString().equals("vectors1.txt") && i == 3) {
      // adjust the distance
      Drive.moveStraightFor(
          (Navigation.distanceBetween(p1, p2) * TILE_SIZE) 
          + TILE_SIZE - (BOX_WIDTH / 2) - CORRECTION * 0.75,
          correction * 4);
    } else {
      // Move straight calculated distance
      Drive.moveStraightFor((Navigation.distanceBetween(p1, p2) 
          * TILE_SIZE) + TILE_SIZE - (BOX_WIDTH / 2) - CORRECTION,
          correction);
    }
    // output the average torque to move the block
    System.out.println("Average torque for block " + i + ": " + Torquemeter.getAverage());

    // store the average torque
    torqueAverages[i - 1] = Torquemeter.getAverage();

    // stop the thread
    Torquemeter.terminate();

    // back up so that pushing bar of the robot does not collide with block
    OdometerCorrection.correctPosition(p1, p2);
  }

  /**
   * Evaluate if there is a block in the trajectory of 
   * the robot and returns block index. Returns -1 if there is no
   * block
   * 
   * @param p1 : Point of initial position of robot
   * @param p2 : Point of final pos of robot
   * @return int -1 if there is no block, and if there is, it returns the index
   */
  public static int blockInPath(Point p1, Point p2, Point[] obstacles) {
    // Point[] blockPositions = Main.currentBlockPosition;
    double y = p2.y - p1.y;
    double x = p2.x - p1.x;

    x = (x == 0) ? 0.01 : x;

    // Using equation line equation ax+By+c=0
    double a = y / x;
    double c = p1.y - (a * p1.x);
    double b = -1;
    double threshold = (TILE_SIZE / 2) * Math.sqrt(2) + ((Resources.BASE_WIDTH / TILE_SIZE) / 2);

    for (int i = 0; i < obstacles.length; i++) {
      if (obstacles[i] == null) {
        break;
      }
      Point block = obstacles[i];

      double numerator = Math.abs((a * block.x) + (b * block.y) + c);

      double denumerator = Math.sqrt((a * a) + (b * b));

      double distanceLinePoint = numerator / denumerator;

      double minX = Math.min(p1.x, p2.x);
      double maxX = Math.max(p1.x, p2.x);
      double minY = Math.min(p1.y, p2.y);
      double maxY = Math.max(p1.y, p2.y);

      // check for the closest point to see if the robot will hit a block
      if (block.x > (minX - threshold) 
          && (block.x < (maxX + threshold)) && (block.y > (minY - threshold))
          && (block.y < (maxY + threshold)) && distanceLinePoint < threshold) {
        return i;
      }
    }
    // no blocks hit
    return -1;
  }

  /**
   * Determine the correction need to avoid a
   *  certain block while reaching the next block to be moved.
   * 
   * @param currentPosition : current robot's location
   * @param obstaclePoint : Point of current Obstacle
   * @param angle : the angle perpendicular to the angle of trajectory
   */
  public static void pathCorrection(Point currentPosition, Point obstaclePoint, double angle) {
    double xpos = obstaclePoint.x;
    double ypos = obstaclePoint.y;
    angle = (angle + 90) % 360;

    // calculate the detour point that the robot needs to go to
    double detourPointx = xpos - Math.sin(angle * DEGREE_TO_RAD);
    double detourPointy = ypos - Math.cos(angle * DEGREE_TO_RAD);
    Point detourPoint = new Point(detourPointx, detourPointy);
    if (redTeam == 04) {
      if (!pointSafeForRobot(detourPoint, true, true)) {
        detourPointx = xpos + Math.sin(angle * DEGREE_TO_RAD);
        detourPointy = ypos + Math.cos(angle * DEGREE_TO_RAD);
      }
    }



    // System.out.println("Block Correction needed");

    Navigation.travelTo(detourPoint);


  }

  /**
   * Travels to the waypoint corresponding to currentPointIndex,
   *  using Find Path to find the appropriate path.
   */
  public static ArrayList<Point> travelToWaypointObstacle(Point destination,
      boolean setIslandPoints) {
    if (setIslandPoints) {
      setIslandWayPointsToGoodPoints(false);
    } else {
      setZonePointsToGoodPoints();
    }
    Point currentLocation = new Point(odometer.getXyt()[0] / TILE_SIZE,
        odometer.getXyt()[1] / TILE_SIZE);
    ArrayList<Point> path = findPath(currentLocation, destination, false, false);


    if (path == null) {
      // ObjectDetection.currentPointIndex ++;
      return null;
    }
    path.remove(0);
    Point point = null;
    boolean straightenX = false;
    boolean straightenY = false;
    for (int j = 0; j < path.size(); j++) {
      point = path.get(j);
      if (ObjectDetection.currentPointIndex == 0) {
        Navigation.turnBySearch(25);
        Navigation.turnBySearch(-50);
        Navigation.turnBySearch(25);
      }

      if (straightenX) {
        double x = odometer.getXyt()[0];
        if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 90, 6)) {
          moveTillLine();
          x = x / TILE_SIZE;

          Drive.moveStraightFor(0.04);
          odometer.setX((((int) x) + 1) * TILE_SIZE);
          odometer.setTheta(90);
        } else if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 270, 6)) {
          moveTillLine();
          x = x / TILE_SIZE;

          Drive.moveStraightFor(0.04);
          odometer.setX((((int) x)) * TILE_SIZE);
          odometer.setTheta(270);
        }
        straightenX = false;
      } else if (straightenY) {
        double y = odometer.getXyt()[1];
        if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 0, 6)
            || OdometerCorrection.approximatelyEqual(odometer.getTheta(), 360, 6)) {
          moveTillLine();
          y = y / TILE_SIZE;

          Drive.moveStraightFor(0.04);
          odometer.setY((((int) y) + 1) * TILE_SIZE);
          odometer.setTheta(0);
        } else if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 180, 6)) {
          moveTillLine();
          y = y / TILE_SIZE;

          Drive.moveStraightFor(0.04);
          odometer.setTheta(180);
          odometer.setY((((int) y)) * TILE_SIZE);
        }
        straightenY = false;
      } else {
        Navigation.travelToSearch(point);
      }
      if (j + 1 < path.size()) {
        if (Navigation.getDestinationAngle(point, path.get(j + 1)) == 0) {
          if ((int) (path.get(j + 1).x) == path.get(j + 1).x && point.x != path.get(j + 1).x) {
            straightenX = true;
          } else if ((int) (path.get(j + 1).y) == path.get(j + 1).y 
              && point.y != path.get(j + 1).y) {
            straightenY = true;
          }
        }
      }
      if (ObjectDetection.oneObstacle) {
        ObjectDetection.currentPointIndex--;
        return null;
      }
    }
    return path;




  }

  /**
   * Sets the possibleGoodPoints to the valid island points.
   * 
   * @param checkBlock Boolean that is true if you 
   *        want to eliminate the points where the current block is.
   */
  public static void setIslandWayPointsToGoodPoints(Boolean checkBlock) {
    ArrayList<Point> allIslandPoints = new ArrayList<Point>(50);
    for (double i = island.ll.x + 0.5; i < island.ur.x; i = i + 0.5) {
      for (double j = island.ll.y + 0.5; j < island.ur.y; j = j + 0.5) {
        allIslandPoints.add(new Point(i, j));
      }
    }
    possibleGoodPoints = new ArrayList<Point>();
    for (Point point : allIslandPoints) {
      if (pointSafeForRobot(point, checkBlock, true)) {
        possibleGoodPoints.add(point);
      }
    }
  }
  
  /**
   * Sets the possibleGoodPoints to the valid start zone points.
   */
  public static void setZonePointsToGoodPoints() {
    ArrayList<Point> allStartingZonePoints = new ArrayList<Point>(50);
    for (double i = ObjectDetection.startingZone.ll.x
        + 0.5; i < ObjectDetection.startingZone.ur.x; i = i + 0.5) {
      for (double j = ObjectDetection.startingZone.ll.y
          + 0.5; j < ObjectDetection.startingZone.ur.y; j = j + 0.5) {
        allStartingZonePoints.add(new Point(i, j));
      }
    }
    possibleGoodPoints = new ArrayList<Point>();
    for (Point point : allStartingZonePoints) {
      if (pointSafeForRobot(point, false, false)) {
        possibleGoodPoints.add(point);
      }
    }
  }

  /**
   * Pushes the found block to the front of the bin, then puts
   *  the robot behind the line in front of the bin.
   * 
   * @return skipBlock when no path was found, interrupted when interrupted, and success on success
   */
  public static String pushBlock() {
    setIslandWayPointsToGoodPoints(false);
    Drive.moveStraightFor(-0.15);
    ArrayList<Point> blockPath =
        findPath(ObjectDetection.currentApproximateBlockPosition,
            ramp.blockDestination, true, false);
    if (blockPath == null) {
      return "skipBlock";
    }
    for (int i = 0; i < blockPath.size() - 1; i++) {
      setIslandWayPointsToGoodPoints(true);
      Point destination = getPushingPoint(blockPath.get(i), blockPath.get(i + 1));

      Point currentLocation = new Point(odometer.getXyt()[0] / TILE_SIZE,
          odometer.getXyt()[1] / TILE_SIZE);
      ArrayList<Point> path = findPath(currentLocation, destination, false, true);
      if (path == null) {
        return "skipBlock";
      }

      for (int j = 0; j < path.size(); j++) { // Go to pushing point

        currentLocation = new Point(odometer.getXyt()[0] / TILE_SIZE,
            odometer.getXyt()[1] / TILE_SIZE);
        Point point = path.get(j);

        if (Navigation.distanceBetween(currentLocation, point) > 0.05) {
          Navigation.travelToSearch(point);
        }
        if (ObjectDetection.oneObstacle) {
          return "interrupted";
        }

      }
      double initialObstacleDetectionRangeValue = ObjectDetection.obstacleDetectionRange;
      ObjectDetection.obstacleDetectionRange = 0.4;
      Navigation.travelToSearch(getEndPointBlock(blockPath.get(i), blockPath.get(i + 1)));
      ObjectDetection.obstacleDetectionRange = initialObstacleDetectionRangeValue;
      ObjectDetection.currentApproximateBlockPosition = blockPath.get(i + 1);
      if (ObjectDetection.oneObstacle) { // If it got cancelled, update the block position to
        // the actual block position

        double x = odometer.getXyt()[0] / TILE_SIZE
            + Math.sin(odometer.getTheta() * DEGREE_TO_RAD) / 2;

        double y = odometer.getXyt()[1] / TILE_SIZE
            + Math.cos(odometer.getTheta() * DEGREE_TO_RAD) / 2;
        ObjectDetection.currentApproximateBlockPosition = new Point(x, y);
      }
      Drive.moveStraightFor(-0.15);

      odometer.printPosition();


    }
    Navigation.travelToSearch(ramp.robotDestination1);
    Navigation.travelToSearch(ramp.robotDestination2);
    return "success";


  }

  /**
   * Resets the possibleGoodPoints arrayList
   *  to the waypoints of the searchzone that the robot can go to.
   */
  public static void resetGoodPoints() {
    possibleGoodPoints = new ArrayList<Point>();
    for (Point point : ObjectDetection.wayPoints) {
      if (pointSafeForRobot(point, true, true)) {
        possibleGoodPoints.add(point);
      }
    }
  }

  /**
   * Obtains the Point the robot should go to in order
   *  to push the block from startPoint to endPoint.
   * 
   * @param startPoint The Point the block is currently at.
   * @param endPoint The point the block is going to go.
   * @return The point the robot needs to travel to in order to push the block.
   */
  public static Point getPushingPoint(Point startPoint, Point endPoint) {
    Double angle = Navigation.getDestinationAngle(startPoint, endPoint);
    Double oppositeAngle = (angle + 180) % 360;


    // get the destination values of X and Y
    double destinationX = (startPoint.x) + (Math.sin(oppositeAngle * DEGREE_TO_RAD)) / 2;
    double destinationY = startPoint.y + Math.cos(oppositeAngle * DEGREE_TO_RAD) / 2;

    Point destination = new Point(destinationX, destinationY);
    return destination;
  }
  
  /**
   * Returns the point the robot needs to travel to in oder to push the block to the endpoint.
   * @param startPoint Point the block starts at.
   * @param endPoint Point the block ends at.
   * @return
   */
  public static Point getEndPointBlock(Point startPoint, Point endPoint) {
    Double angle = Navigation.getDestinationAngle(startPoint, endPoint);
    Double oppositeAngle = (angle + 180) % 360;


    // get the destination values of X and Y
    double destinationX = (endPoint.x) + (Math.sin(oppositeAngle * DEGREE_TO_RAD)) / 3;
    double destinationY = endPoint.y + Math.cos(oppositeAngle * DEGREE_TO_RAD) / 3;

    Point destination = new Point(destinationX, destinationY);
    return destination;
  }

  /**
   * Finds a safe path from currentLocation using Breadth-Search-First Algorithm.
   * 
   * @param currentLocation the start location of the path
   * @param destination the end location of the path
   * @param isABlock is this path done for a block?
   *        This makes the method check if the path for the block has points
   *        that the robot can push from.
   * @param checkBlock is this method going to check if the path
   *        goes through the current location of the block?
   * @return the path in an arrayList. the first point is the currentLocation of the robot.
   */
  public static ArrayList<Point> findPath(Point currentLocation,
      Point destination, boolean isABlock,
      boolean checkBlock) {
    ArrayList<Point> pathPoints = new ArrayList<Point>();
    ArrayList<Point> visitedPoints = new ArrayList<Point>();
    Queue<Node> queue = new ArrayDeque<Node>();
    Node current = new Node(currentLocation, null);
    queue.add(current);
    outerloop: while (!queue.isEmpty()) {
      current = queue.remove();

      for (Point point : possibleGoodPoints) {
        if (!visitedPoints.contains(point) 
            && Navigation.distanceBetween(point, current.currentPoint) < 0.6
            && safeForRobotPushing(point, current.currentPoint, isABlock, checkBlock)) {
          if (point.equals(destination)) {
            current = new Node(point, current);
            break outerloop;
          }
          if (Navigation.distanceBetween(point, destination) < 0.36) {

            current = new Node(point, current);
            current = new Node(destination, current);
            break outerloop;
          }
          queue.add(new Node(point, current));
          visitedPoints.add(point);
        }
      }
    }
    if (!current.currentPoint.equals(destination)) {
      return null;
    }
    
    while (current != null) {
      pathPoints.add(current.currentPoint);
      current = current.previous;

    }
    Collections.reverse(pathPoints);

    return pathPoints;


  }

  /**
   * Checks if the block can be pushed from startPoint to endPoint safely,
   *  by checking the point the robot needs to
   * travel to.
   * 
   * @param endPoint The end location of the block.
   * @param startPoint The start location of the block.
   * @param isABlock Makes sure that this is a block. If is not, it returns true.
   * @param checkBlock Checks if you want to consider the current location of the block.
   * @return
   */
  public static boolean safeForRobotPushing(Point endPoint, Point startPoint,
      boolean isABlock, boolean checkBlock) {
    if (!ObjectDetection.hasBlock || !isABlock) {

      return true;
    } else {
      Point destination = getPushingPoint(startPoint, endPoint);
      boolean safe = pointSafeForRobot(destination, checkBlock, true);
      return safe;
    }

    
  }

  /**
   * Checks if the point is safe for robot, comparing the point to a multitude of parameters.
   * 
   * @param point Point that needs to be checked
   * @param checkBlock true if you want to check the current location of the block.
   * @param checkIsland true if you want to check the point to make sure its in the island.
   * @return
   */
  public static boolean pointSafeForRobot(Point point, boolean checkBlock,
      boolean checkIsland) {
    // change, check all obstacles

    if (checkIsland && (point.x < island.ll.x + 0.4 
        || point.x > island.ur.x - 0.4 || point.y < island.ll.y + 0.4
        || point.y > island.ur.y - 0.4)) {
      return false;
    }
    if (checkIsland && (point.y < ramp.walledOffRegion.ur.y + 0.4 
        && point.y > ramp.walledOffRegion.ll.y - 0.4
        && point.x < ramp.walledOffRegion.ur.x + 0.4 
        && point.x > ramp.walledOffRegion.ll.x - 0.4)) {
      return false;
    }
    if (checkBlock && ObjectDetection.currentApproximateBlockPosition != null
        && Navigation.distanceBetween(point, 
            ObjectDetection.currentApproximateBlockPosition) < 0.4) {

      return false;
    }
    for (Point obstacle : ObjectDetection.blocksGivenUpOn) {
      if (Navigation.distanceBetween(obstacle, point) < 0.7) {
        return false;
      }
    }

    for (Point obstacle : ObjectDetection.obstacles) {
      if (Navigation.distanceBetween(obstacle, point) < 0.85) {
        return false;
      }
    }
    return true;
  }
}

