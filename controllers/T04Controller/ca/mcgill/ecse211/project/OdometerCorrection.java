package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.LightLocalizer.moveTillLine;
import static ca.mcgill.ecse211.project.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.project.Resources.SLOW_SPEED;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.island;
import static ca.mcgill.ecse211.project.Resources.odometer;

import ca.mcgill.ecse211.playingfield.Point;



public class OdometerCorrection {

  /**
   * Correct the position of the odometer to the closest point, so it is 
   * as accurate as possible as to where we are suppose to be.
   * @param p1 The first point to use in the correction
   * @param p2 The second point to in the correction (must be recorded after the first)
   */
  public static void correctPosition(Point p1, Point p2) {
    Drive.moveBack();
    double x = odometer.getXyt()[0];
    double y = odometer.getXyt()[1];
    double theta = odometer.getXyt()[2];
   
    //if x is equal
    if (p1.x == p2.x) {
      //if p1 y coordinate is less than p2 y
      if (p1.y < p2.y) {
        //robot facing forward
        y = p2.y - 1;
        theta = 0;
      } else {
        //robot facing backwards
        y = p2.y + 1;
        theta = 180;
      }
      x = p1.x;
    }

    //If the Y's are equal
    if (p1.y == p2.y) {
      //P1 x is less than p2 x
      if (p1.x < p2.x) {
        //robot is facing right
        x = p2.x - 1;
        theta = 90;
      } else {
        //robot is facing left
        x = p2.x + 1;
        theta = 270;
      }
      y = p1.y;
    }

    //set the odometer with the updated values
    odometer.setXyt(x * TILE_SIZE, y * TILE_SIZE, theta);
    
  }
  
  /**
   * Set the odometer to a specific waypoint.
   * @param current the waypoint that the odometer will be set to
   */
  public static void correctOdometer(Point current) {
    odometer.setXyt(current.x * TILE_SIZE, current.y * TILE_SIZE, odometer.getXyt()[2]);
  }
  //Didn't test it but it looks pretty good to me.
  /**
   * Advances the robot to the line and corrects the theta as well as one of the directions.
   */
  
  public static void lineAndCorrect() {
    LightLocalizer.moveTillLine();
    double theta = odometer.getTheta();
    theta = theta + 20;
    int turns = (int) Math.round(theta / 90);
    odometer.setTheta(turns * 90);
    double x = odometer.getXyt()[0];
    double y = odometer.getXyt()[1];
    x = x / TILE_SIZE;
    y = y / TILE_SIZE;
    if (turns == 0 || turns == 2) {
      y = Math.round(y);
    } else {
      x = Math.round(x);
    }
    System.out.print("Before line and correct");
    odometer.printPosition();
    odometer.setXyt(x, y, theta);
    System.out.print("After line and correct");
    odometer.printPosition();

  }
  
  /**
   * Check odometer coordinates before crossing bridge.
   * @param tunnelExit coordinate of exit of bridge
   * @param tunnelEntrance coordinate of entrance of bridge
   */
  public static void checkOdometerToCrossBridge(
      Point tunnelExit, Point tunnelEntrance) {

    double minAngle = Navigation.getDestinationAngle(tunnelExit, tunnelEntrance);

    System.out.println(minAngle);
    Navigation.turnBy(Navigation.minimalAngle(odometer.getTheta(), minAngle));

    setToNearestLine(tunnelExit, minAngle);

  }

  /**
   * Check whether within range given passed values.
   * @param min Min passed value.
   * @param max Max passed value.
   * @param value Given value.
   * @return boolean which gives if given value is withen min and max value.
   */
  public static boolean withinRange(double min, double max, double value) {
    return ((value < max) && (value > min));
  }

  /**
   * Check if 2 given values are close enough to be considered equal, given threshold.
   * @param actualValue passsed value in question.
   * @param idealValue theoretical or ideal value
   * @param threshold threshold by which to compare value to.
   * @return boolean which states wether or not to be considered equal.
   */
  public static boolean approximatelyEqual(double actualValue, double idealValue, 
      double threshold) {

    return (Math.abs(actualValue - idealValue) <= threshold);
  }

  /**
   * Sets the odometer to the nearest line.
   * 
   * @param point Current Location
   * @param minAngle Minimal angle to line
   */
  public static void setToNearestLine(
      Point point, double minAngle) {

    double currentTheta = odometer.getTheta();
    double threshold = 5;

    double resetY = -1;
    double resetX = -1;
    double resetTheta = -1;

    if (approximatelyEqual(currentTheta, 90.0d, threshold) 
        || approximatelyEqual(currentTheta, 270.0d, threshold)) {

      if (withinRange(island.ll.y, island.ur.y, (odometer.getXyt())[1] / TILE_SIZE + 1.5)) {
        if (approximatelyEqual(currentTheta, 90.0d, threshold)) {
          Navigation.turnBy(90);
          resetTheta = 180;
        } else {
          Navigation.turnBy(-90);
          resetTheta = 180;
        }
        resetY = (int) (point.y + 1);

      } else {
        if (approximatelyEqual(currentTheta, 90.0d, threshold)) {
          Navigation.turnBy(-90);
          resetTheta = 0;
        } else {
          Navigation.turnBy(90);
          resetTheta = 0;
        }
        resetY = ((int) (point.y - 1)) + 1;
      }

    } else if (approximatelyEqual(currentTheta, 0.0d, threshold) 
        || approximatelyEqual(currentTheta, 360.0d, threshold)
        || approximatelyEqual(currentTheta, 180.0d, threshold)) {
      if (withinRange(island.ll.x, island.ur.x, (odometer.getXyt())[0] / TILE_SIZE + 1.5)) {
        if (approximatelyEqual(currentTheta, 0.0d, threshold) 
            || approximatelyEqual(currentTheta, 360.0d, threshold)) {
          Navigation.turnBy(-90);
          resetTheta = 270;
        } else {
          Navigation.turnBy(90);
          resetTheta = 270;
        }
        resetX = (int) (point.x + 1);
      } else {
        if (approximatelyEqual(currentTheta, 0.0d, threshold) 
            || approximatelyEqual(currentTheta, 360.0d, threshold)) {
          Navigation.turnBy(90);
          resetTheta = 90;
        } else {
          Navigation.turnBy(-90);
          resetTheta = 90;
        }
        resetX = ((int) (point.x - 1)) + 1;
      }
    }

    Drive.moveStraightFor(-1 * TILE_SIZE);
    Drive.setSpeed(SLOW_SPEED + 150);
    LightLocalizer.moveStraightSameSpeed();
    moveTillLine();
    Drive.setSpeed(FORWARD_SPEED);
    Navigation.moveStraightFor(0.13);
    // System.out.println("Y " + resetY + " X " + resetX);

    if (resetY > 0) {
      odometer.setY(resetY * TILE_SIZE);
      Navigation.moveStraightFor(Math.abs(resetY - point.y));
    } else if (resetX > 0) {
      odometer.setX(resetX * TILE_SIZE);
      Navigation.moveStraightFor(Math.abs(resetX - point.x));
    }

    if (resetTheta > 0) {
      odometer.setTheta(resetTheta);
    }

    odometer.printPosition();

    Navigation.turnBy(Navigation.minimalAngle(odometer.getTheta(), minAngle));

    Drive.setSpeed(SLOW_SPEED + 150);
    LightLocalizer.moveStraightSameSpeed();
    moveTillLine();

    resetX = -1;
    resetY = -1;

    // currentTheta = odometer.getTheta();

    odometer.printPosition();
    if (approximatelyEqual(currentTheta, 90.0d, threshold)) {
      resetX = point.x + 0.5;
      resetTheta = 90;
    } else if (approximatelyEqual(currentTheta, 270.0d, threshold)) {
      resetX = point.x - 0.5;
      resetTheta = 270;
    } else if (approximatelyEqual(currentTheta, 180.0d, threshold)) {
      resetY = point.y - 0.5;
      resetTheta = 180;
    } else if (approximatelyEqual(currentTheta, 0.0d, threshold)
        || approximatelyEqual(currentTheta, 360.0d, threshold)) {
      resetY = point.y + 0.5;
      resetTheta = 0;
    }

    Drive.moveStraightFor(0.04);
    odometer.printPosition();

    if (resetY > 0) {
      odometer.setY(resetY * TILE_SIZE);
    } else if (resetX > 0) {
      odometer.setX(resetX * TILE_SIZE);
    }

    if (resetTheta > 0) {
      odometer.setTheta(resetTheta);
    }
    System.out.print("Odometer after reset: ");
    odometer.printPosition();

  }
  
  /**
   * Method to verify the odometer.
   * @param exitPoint Exit Point of the tunnel
   */
  public static void checkOdometerAfterCrossBridge(
      Point exitPoint) {
    double minAngle = odometer.getTheta();
    setToNearestLine(exitPoint, minAngle);

  }

}

