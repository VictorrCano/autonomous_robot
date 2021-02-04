package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.LightLocalizer.moveTillLine;
import static ca.mcgill.ecse211.project.Resources.*;

import java.util.*;

import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.Region;
import simlejos.hardware.ev3.LocalEV3;


/**
 * Class to run object detection and search for both blocks and Obstacles.
 *
 */
public class ObjectDetection implements Runnable {

  /** Boolean to determine if the ObjectDectection thread is running. */
  public static boolean isRunning = true;

  /** Boolean to determine if the robot is in its corresponding search zone. */
  public static boolean inSearchZone = false;

  /** Array for storing samples from front color sensor. */
  public static float[] currentValue = new float[5];

  /** List of waypoints to cover the entire inner search zone. */
  public static ArrayList<Point> wayPoints = new ArrayList<Point>();

  /** Index used to obtain all waypoints of the search zone. */
  public static int index = 0;

  /** Direction of robot's path, 0 for north to south,
   *  1 for south to north. */
  public static int direction = 0;

  /** Boolean used to determine the
   *  robot's arrival to the search zone. */
  public static boolean arrived = false;
  public static ArrayList<Point> obstacles = new ArrayList<Point>();
  public static ArrayList<Point> blocksGivenUpOn = new ArrayList<Point>();
  public static boolean isFirstObstacle = false;

  public static boolean hasBlock = false;
  public static int currentPointIndex;
  public static Point currentApproximateBlockPosition;
  public static boolean rampAvoided = false;
  public static double obstacleDetectionRange = 0.15;
  public static Region startingZone;
  public static ArrayList<Point> startZonePath;

  /**
   * Flag used to detect if an obstacle is present.
   */
  public static boolean oneObstacle = false;

  /** Singleton instantiation of object detection thread. */
  public static synchronized ObjectDetection getObjectDetection() {
    if (objectDetection == null) {
      objectDetection = new ObjectDetection();
    }
    return objectDetection;
  }

  public static boolean justBackedUp = false;


  /**
   * Method to run the object detection for looking for both blocks and obstacles.
   */
  @Override
  public void run() {
    float[] sampleUsTop = new float[usSensorTop.sampleSize()];
    float[] sampleUs = new float[usSensor.sampleSize()];
    while (isRunning) {

      usSensorTop.fetchSample(sampleUsTop, 0);
      usSensor.fetchSample(sampleUs, 0);

      if (!thereIsAWall(sampleUsTop[0]) && sampleUsTop[0] 
          <= obstacleDetectionRange && !isATunnel()) { // change this
        odometer.printPosition(); // experimentally.
        avoidObstacle();

      } else if (sampleUs[0] < 0.07 && !thereIsAWall(sampleUs[0]) && sampleUsTop[0] > 0.1
          && !hasBlock && !isATunnel()) { // Start of block
        // detection
        hasBlock = true;
        oneObstacle = true;

      }
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {

        e.printStackTrace();
      }

      // Get robot's current position to check if it has entered serach zone.
      double x = odometer.getXyt()[0] / TILE_SIZE;
      double y = odometer.getXyt()[1] / TILE_SIZE;
      Point point = new Point(x, y);
      if (!arrived && reachedSearchZone(point)) {
        arrived = true;
        beep();
      }
    }
  }

  /**
   * Method to check whether we have a tunnel.
   * @return boolean which returns whether or not is a tunnel.
   */
  private boolean isATunnel() {

    if (!IsOnIsland()) {
      return false;
    }

    double[] positions = odometer.getXyt();



    double x = odometer.getXyt()[0] / TILE_SIZE
            + (Math.sin(positions[2] * DEGREE_TO_RAD) * (0.1 + obstacleDetectionRange) / TILE_SIZE);
    double y = odometer.getXyt()[1] / TILE_SIZE
        + (Math.cos(positions[2] * DEGREE_TO_RAD) * (0.1 + obstacleDetectionRange) / TILE_SIZE);

    if (OdometerCorrection.withinRange(tnr.ll.x, tnr.ur.x, x)
        && OdometerCorrection.withinRange(tnr.ll.y, tnr.ur.y, y)) {

      return true;
    }

    if (OdometerCorrection.withinRange(tng.ll.x, tng.ur.x, x)
        && OdometerCorrection.withinRange(tng.ll.y, tng.ur.y, y)) {

      return true;
    }


    return false;
  }

  /**
   * Method to check wether robot on island.
   * @return boolean which return if on island.
   */
  private boolean IsOnIsland() {
    double[] positions = odometer.getXyt();
    if (OdometerCorrection.withinRange(island.ll.x, island.ur.x, positions[0] / TILE_SIZE)
        && OdometerCorrection.withinRange(island.ll.y, island.ur.y, positions[1] / TILE_SIZE)) {
      return true;
    }
    return false;
}

  /**
   * Method to check if wall has been found.
   * @param usSample ultrasonic readings to be checked for wall 
   * @return boolean of if a wall is present
   */
  private boolean thereIsAWall(float usSample) {
    boolean isWall = false;
    double[] positions = odometer.getXyt();
    Region searchZone;
    if (usSample < 0.1) {
      if (redTeam == 04) {
        searchZone = szr;
      } else {
        searchZone = szg;
      }

      if (positions[2] >= 340 || positions[2] <= 20) {
        if (searchZone.ur.y - (positions[1] / TILE_SIZE) < 1) {
          isWall = true;
        }
      }

      if (positions[2] <= 110 && positions[2] >= 70) {
        if (searchZone.ur.x - (positions[0] / TILE_SIZE) < 1) {
          isWall = true;
        }
      }
      if (positions[2] <= 200 && positions[2] >= 160) {
        if ((positions[1] / TILE_SIZE) - searchZone.ll.y < 1) {
          isWall = true;
        }
      }
      if (positions[2] <= 290 && positions[2] >= 250) {
        if ((positions[0] / TILE_SIZE) - searchZone.ur.x < 1) {
          isWall = true;
        }
      }

      return isWall;

    }
    return false;
  }

  /**
   * Move bock to front of bin, at bottom of ramp.
   */
  public static void pushBlockToFrontOfBin() {

    // extra check to make sure we are pushing a block
    oneObstacle = true;
    try {
      Thread.sleep(40);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    oneObstacle = false;
    hasBlock = true;
    beep();
    double x = odometer.getXyt()[0] / TILE_SIZE + Math.sin(odometer.getTheta() * DEGREE_TO_RAD) / 2;

    double y = odometer.getXyt()[1] / TILE_SIZE + Math.cos(odometer.getTheta() * DEGREE_TO_RAD) / 2;
    currentApproximateBlockPosition = new Point(x, y);

    String result = BlockMover.pushBlock();

    while (oneObstacle) {
      while (oneObstacle) {
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
      BlockMover.pushBlock();
    }
    hasBlock = false;

    // call the methods from main to control block pushing


    oneObstacle = false;
    if (result.equalsIgnoreCase("success")) {
      pushBlockIntoBin();
    }
    if (result.equalsIgnoreCase("SkipBlock")) {
      blocksGivenUpOn.add(currentApproximateBlockPosition);
      currentApproximateBlockPosition = null;
    }

    // By calling this method from main as soon as initialize (i.e. searching for
    // a block) ends it will move to push the block to the bin while still running
    // the object detection to make sure we avoid obstacles
    // as such, for all navigation use the travelToForSearch() method in Navigation
    // for all movements of the Robot.

  }
  
  /**
   * Make robot output sound.
   */
  private static void beep() {
    try {
      Thread.sleep(200);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }
    LocalEV3.audio.beep();
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    LocalEV3.audio.beep();
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    LocalEV3.audio.beep();
  }
  
  /**
   * Method called when the block is pushed into the bin.
   */
  public static void pushBlockIntoBin() {

    Drive.moveStraightFor(0.1);
    moveTillLine();
    boolean noBlock = false;
    int numberLoops = 0;
    isRunning = false;
    Drive.moveStraightFor(0.33);

    Torquemeter.initialize();
    new Thread(torquemeter).start();

    leftMotor.forward();
    rightMotor.forward();
    int prevLeftTachoCount = 0;
    int prevRightTachoCount = 0;
    float[] sampleUs = new float[usSensor.sampleSize()];
    double prevSample = 0;
    while (true) {

      usSensor.fetchSample(sampleUs, 0);

      if (sampleUs[0] > 0.09 && sampleUs[0] >= prevSample) {
        if (numberLoops <= 2) {
          noBlock = true;
        }
        leftMotor.stop();
        rightMotor.stop();

      }
      prevSample = sampleUs[0];
      if (leftMotor.getTachoCount() == prevLeftTachoCount 
          && rightMotor.getTachoCount() == prevRightTachoCount) {

        break;
      }

      prevLeftTachoCount = leftMotor.getTachoCount();
      prevRightTachoCount = rightMotor.getTachoCount();
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {

        e.printStackTrace();
      }
      numberLoops++;
    }

    try { // pause for 2 seconds
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    Torquemeter.decideOnWeightOfBlock();
    if (noBlock) {
      Drive.moveStraightFor(-0.67);
    } else {
      Drive.moveStraightFor(-0.73);
    }
    double x = odometer.getXyt()[0];
    double y = odometer.getXyt()[1];

    odometer.printPosition();
    hasBlock = false;
    oneObstacle = false;
    Navigation.setSpeed(SLOW_SPEED + 150);
    if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 0, 6)
        || OdometerCorrection.approximatelyEqual(odometer.getTheta(), 360, 6)) {
      moveTillLine();
      y = y / TILE_SIZE;

      odometer.setY((((int) y) + 1) * TILE_SIZE);
    } else if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 90, 6)) {
      moveTillLine();
      x = x / TILE_SIZE;

      odometer.setX((((int) x) + 1) * TILE_SIZE);
    } else if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 270, 6)) {
      moveTillLine();
      x = x / TILE_SIZE;

      odometer.setX((((int) x)) * TILE_SIZE);
    } else if (OdometerCorrection.approximatelyEqual(odometer.getTheta(), 180, 6)) {
      moveTillLine();
      y = y / TILE_SIZE;

      odometer.setY((((int) y)) * TILE_SIZE);
    }
    Navigation.setSpeed(FORWARD_SPEED);
    // leftMotor.stop();
    // rightMotor.stop();
    odometer.printPosition();


  }
  
  /**
   * Method which predicts obstacle location baded on UltraSonic readings.
   */
  private void predictObstacleLocation() {
    float[] sensorValue = new float[usSensorTop.sampleSize()];
    usSensorTop.fetchSample(sensorValue, 0);
    double initialDistance = sensorValue[0];
    double currentDistance = initialDistance;

    int counter = 0;
    double initialAngle = odometer.getTheta();
    while (currentDistance < initialDistance * 1.4 && counter < 15) {
      Navigation.turnBy(2);
      usSensorTop.fetchSample(sensorValue, 0);
      currentDistance = sensorValue[0];
      counter++;
    }
    double alphaAngle = odometer.getTheta();

    Navigation.turnBy(-2 * (counter - 2));
    counter = 0;
    usSensorTop.fetchSample(sensorValue, 0);
    usSensorTop.fetchSample(sensorValue, 0);
    currentDistance = sensorValue[0];
    while (currentDistance < initialDistance * 1.4 && counter < 15) {
      Navigation.turnBy(-2);
      usSensorTop.fetchSample(sensorValue, 0);
      currentDistance = sensorValue[0];
      counter++;
    }
    double betaAngle = odometer.getTheta();
    if (alphaAngle < betaAngle) {
      alphaAngle += 360;
    }
    double angle;
    if (Math.abs(alphaAngle - betaAngle) < 80) {
      angle = ((alphaAngle + betaAngle) / 2) % 360;
    } else {
      angle = initialAngle;
    }
    double x = odometer.getXyt()[0] / TILE_SIZE
        + (Math.sin(angle * DEGREE_TO_RAD) * (0.1 + obstacleDetectionRange) / TILE_SIZE);
    double y = odometer.getXyt()[1] / TILE_SIZE
        + (Math.cos(angle * DEGREE_TO_RAD) * (0.1 + obstacleDetectionRange) / TILE_SIZE);

    Point obstacle = new Point(x, y);


    obstacles.add(obstacle);
    odometer.printPosition();
    Navigation.turnTo(angle);


  }
  
  /**
   * Method called when an obstacle is detected in run.
   */
  private void avoidObstacle() {
    oneObstacle = true;
    Navigation.setSpeed(FORWARD_SPEED);


    // Trying to predict where exactly the obstacle is seems near impossible to me.
    // Putting 1 tile ahead of the robot works for rectangles, but may be overkill for other shapes
    // where the outside of the obstacle is closer to the center.
    // My fear for this is that we would mark off some waypoints by being overtly careful,
    // and that these waypoints would have a block. However, since there are many blocks, being
    // very careful isnt so bad. Also dont fast forward, it screws things up.

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    predictObstacleLocation();
    Navigation.moveStraightFor(-0.35);
    odometer.printPosition();
    justBackedUp = true;
    oneObstacle = false;
  }

  /**
   * Method to initalize the variables needed for object detection
   *  and to initiate a regulated search pattern.
   */
  public static void initalize() {
    isRunning = true;
    inSearchZone = true;
    oneObstacle = false;
    hasBlock = false;

    Point topLeft = null;
    Region sz = BlockMover.searchZone;

    topLeft = new Point(sz.ll.x + 1.0 / 2, sz.ur.y - 1.0 / 2);

    Point initial = topLeft;
    wayPoints.add(initial);
    getWaypoints();


    for (currentPointIndex = 0; currentPointIndex < wayPoints.size(); currentPointIndex++) {
      Point point = wayPoints.get(currentPointIndex);

      if (isRunning) {

        odometer.printPosition();

        if (currentPointIndex % 2 == 0 && !pointInBin(point) && !justBackedUp && !atBorder()) {
          Navigation.turnBySearch(25);
          Navigation.turnBySearch(-50);
          Navigation.turnBySearch(25);
        }

        if (BlockMover.pointSafeForRobot(point, true, true)) {
          BlockMover.travelToWaypointObstacle(point, true);

          if (!oneObstacle && !hasBlock && !justBackedUp && !pointInBin(point)) {

            odometer.setX(point.x * TILE_SIZE);
            odometer.setY(point.y * TILE_SIZE);
          } else if (justBackedUp) {
            justBackedUp = false;
          }
        }

        if (hasBlock) {
          pushBlockToFrontOfBin();

        }

        while (oneObstacle || hasBlock) {

          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }
  }

  /**
   * Method used to determine when the robot enters the search zone and beep three times.
   *
   * @param point : current location of robot.
   * @return true if point is within the search zone.
   */
  public static boolean reachedSearchZone(Point point) {

    Region sz = BlockMover.searchZone;
    if (sz.ll.x <= point.x && point.x <= sz.ur.x && sz.ll.y <= point.y && point.y <= sz.ur.y) {
      return true;
    }
    return false;
  }

  /**
   * Check if the robot is at the boundary of the island.
   *
   * @return true if it is
   */
  public static boolean atBorder() {
    double x = odometer.getXyt()[0] / TILE_SIZE;
    double y = odometer.getXyt()[1] / TILE_SIZE;
    Point current = new Point(x, y);
    Boolean first = (current.x == Main.exitPoint.x && current.y == Main.exitPoint.y);

    if (first || current.y > (BlockMover.searchZone.ur.y - 1)
        || current.y < (BlockMover.searchZone.ll.y + 1)) {
      return true;
    }
    return false;
  }

  /**
   * Sends the robot to the exit Point of the tunnel.
   *
   * @param exitPoint the exit point of the tunnel
   */
  public static void resetToReturnHome(Point exitPoint) {
    odometer.printPosition();

    BlockMover.setIslandWayPointsToGoodPoints(false);
    ArrayList<Point> path = BlockMover.findPath(
        new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE),
        exitPoint, false, false);
    if (path == null || path.size() == 0) {

      Navigation.travelToSearch(exitPoint);
    } else {

      for (Point point : path) {
        Navigation.travelToSearch(point);
      }
    }
    BlockMover.resetGoodPoints();
  }



  /**
   * Travel to a point in the search area without scanning for blocks/obstacles along the way.
   *
   * @param point The point (x,y) that the robot will travel to
   */
  public static void travelToWayPoints(Point point) {
    if (isRunning) {
      Navigation.travelTo(point);
      odometer.setXyt(point.x * TILE_SIZE, point.y * TILE_SIZE, odometer.getTheta());
      // odometer.printPosition();
    }
  }

  /**
   * Get all waypoints in a zig-zag pattern covering every half point in search zone.
   */
  public static void getWaypoints() {

    Region sz = BlockMover.searchZone;
    for (int i = 0; i < (sz.ur.x - sz.ll.x) * 2; i++) {
      for (int j = 0; j < (sz.ur.y - sz.ll.y) * 2; j++) {

        Point current = wayPoints.get(index);
        Point nextWayPoint;

        if (current.x <= (sz.ur.x) && current.y <= (sz.ur.y - 0.5) && current.y >= (sz.ll.y)) {

          if (direction == 0) { // traversing downwards
            nextWayPoint = new Point(current.x, (current.y - 0.5));
          } else {
            nextWayPoint = new Point(current.x, (current.y + 0.5));
          }

          if (nextWayPoint.y < (sz.ll.y + 0.5) && nextWayPoint.x <= (sz.ur.x - 0.5)) {
            nextWayPoint = new Point(current.x + 0.5, current.y);

            wayPoints.add(nextWayPoint);
            direction = 1;
          } else if (nextWayPoint.y > (sz.ur.y - 0.5) && nextWayPoint.x <= (sz.ur.x - 0.5)) {
            nextWayPoint = new Point(current.x + 0.5, current.y);

            wayPoints.add(nextWayPoint);
            direction = 0;
          } else {
            wayPoints.add(nextWayPoint);
          }
          index++;
        }
      }
    }
  }

  /**
   * Check if next waypoint coordinates are within the bounding box of the bin.
   *
   * @Param point: the next waypoint in search zone
   * @return true if it is within the area
   */
  public static boolean pointInBin(Point point) {
    if (redTeam == 04) {
      if (rr.left.x <= point.x && point.x <= rr.right.x
          && rr.left.y <= point.y && point.y <= rr.left.y + 2.0) {
        return true;
      }
      return false;
    } else {
      if (gr.left.x <= point.x && point.x <= gr.right.x
          && gr.left.y <= point.y && point.y <= gr.left.y + 2.0) {
        return true;
      }
      return false;
    }
  }


  /**
   * Initial version of obstacle detection. Searches for any
   *  obstacle on line. Needs to be changed to cover whole area.
   * Consider using US sensor.
   *
   * @param upLeft upper left of the search area
   * @param upRight upper right of the search area
   * @param downRight lower right of the search area
   * @param downLeft lower left of the search area
   */
  public static void searchForObstacle(Point upLeft, Point upRight,
      Point downRight, Point downLeft) {
    double numberOfPointsVisited = 0;

    double totalNumberOfPoints = ((upRight.x - upLeft.x + 1) * (upRight.y - downRight.y + 1));

    for (int i = 0; numberOfPointsVisited < totalNumberOfPoints; i++) {
      Navigation.travelTo(upLeft);
      numberOfPointsVisited++;
      Navigation.travelTo(upRight);
      numberOfPointsVisited = numberOfPointsVisited + (upRight.x - upLeft.x);
      Navigation.travelTo(downRight);
      numberOfPointsVisited = numberOfPointsVisited + (upRight.y - downRight.y);
      Navigation.travelTo(downLeft);
      numberOfPointsVisited = numberOfPointsVisited + (downRight.x - downLeft.x);
      upLeft = new Point(upLeft.x + 1, upLeft.y - 1);
      upRight = new Point(upRight.x - 1, upRight.y - 1);
      downRight = new Point(downRight.x - 1, downRight.y + 1);
      downLeft = new Point(downLeft.x + 1, downRight.y + 1);
    }
  }

  /**
   * Sends the robot to the entryPoint of the tunnel.
   */
  public static void entryInitialize() {

    currentPointIndex = 10;
    leftMotor.setSpeed(FORWARD_SPEED + 200);
    rightMotor.setSpeed(FORWARD_SPEED + 200);
    startZonePath = BlockMover.travelToWaypointObstacle(
        Main.entryPoint, false);
    while (oneObstacle) {
      while (oneObstacle) {
        try {
          Thread.sleep(200);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
      startZonePath
          = BlockMover.travelToWaypointObstacle(Main.entryPoint, false);
    }
  }
}
