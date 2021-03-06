package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;

import ca.mcgill.ecse211.playingfield.Point;

public class Navigation {
  
  /** Do not instantiate this class. */
  private Navigation() {}
  
  /**
   *  Travel to destination searching.
   * @param destination Destination to travel to.
   */
  public static void travelToSearch(Point destination) {
    var xyt = odometer.getXyt();
    var currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    var currentTheta = xyt[2];
    var destinationTheta = getDestinationAngle(currentLocation, destination);
    turnBySearch(minimalAngle(currentTheta, destinationTheta));
    // When in the robot is in search zone, turn off and back on the sensor sampling so it
    // does not takes samples during rotation
    ObjectDetection.isRunning = true;
    
    moveStraightForSearch(distanceBetween(currentLocation, destination));
  }
  
  /**
   *   Method to travel to given destination.
   * @param destination destination to travel to
   */
  public static void travelTo(Point destination) {
    var xyt = odometer.getXyt();
    var currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    var currentTheta = xyt[2];
    var destinationTheta = getDestinationAngle(currentLocation, destination);
    //ObjectDetection.isRunning = false;
    turnBy(minimalAngle(currentTheta, destinationTheta));
    // When in the robot is in search zone, turn off and back on the sensor sampling so it
    // does not takes samples during rotation
    //ObjectDetection.isRunning = true;

    moveStraightFor(distanceBetween(currentLocation, destination));
  }
  
  
  /**
   * Returns the angle that the robot should point towards to face the destination in degrees.
   * @param current Point current
   * @param destination Point destination
   * @return
   */
  public static double getDestinationAngle(Point current, Point destination) {
    return (toDegrees(atan2(destination.x - current.x, destination.y - current.y)) + 360) % 360;
  }
  
  /**
   * Returns the signed minimal angle from the initial angle to the destination angle.
   * 
   * @param initialAngle double initialAngle
   * @param destAngle double destAngle
   * @return
   */
  public static double minimalAngle(double initialAngle, double destAngle) {
    var dtheta = destAngle - initialAngle;
    if (dtheta < -180) {
      dtheta += 360;
    } else if (dtheta > 180) {
      dtheta -= 360;
    }
    return dtheta;
  }
  
  /**
   * calculates distance between 2 points.
   * 
   * @param p1 Point p1 First point to calculate distance from
   * @param p2 Point p2 Second point to calculate distance from
   * @return
   */
  public static double distanceBetween(Point p1, Point p2) {
    var dx = p2.x - p1.x;
    var dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
  }
  

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightForSearch(double distance) {
    setSpeed(FORWARD_SPEED);
    
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    int prevLeftTachoCount = 0;
    int prevRightTachoCount = 0;
    while (true) {
      if (ObjectDetection.oneObstacle) {
        leftMotor.stop();
        rightMotor.stop();

      }

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
      
    }
    
  }
  
  /**
   * Move robot straight for given distance.
   * @param distance Distance to be travelled for.
   */
  public static void moveStraightFor(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), false);
    odometer.printPosition();
  }
  
  /**
   * Moves the robot forward for an indeterminate distance.
   */
  public static void forward() {
    setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }
  
  /**
   * Moves the robot backward for an indeterminate distance.
   */
  public static void backward() {
    setSpeed(FORWARD_SPEED);
    leftMotor.backward();
    rightMotor.backward();
  }
  
  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }
  
  /**
   * Like turnBy but turn can be interrupted.
   * @param angle The angle by which robot should turn, in degrees.
   */
  public static void turnBySearch(double angle) {
    setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), true);
    int prevLeftTachoCount = 0;
    int prevRightTachoCount = 0;
    while (true) {
      if (ObjectDetection.oneObstacle) {
        leftMotor.stop();
        rightMotor.stop();

      }

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
    }
  }
  
  /** Rotates motors clockwise. */
  public static void clockwise() {
    setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
  }
  
  /** Rotates motors counterclockwise. */
  public static void counterclockwise() {
    setSpeed(ROTATE_SPEED);
    leftMotor.backward();
    rightMotor.forward();
  }
  
  /** Stops both motors. This also resets the motor speeds to zero. */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance in degrees
   */
  public static int convertDistance(double distance) {
    return (int) toDegrees(distance / WHEEL_RAD);
  }

  /**
   * Converts input angle to total rotation of each wheel needed to rotate robot by that angle.
   * 
   * @param angle the input angle in degrees
   * @return the wheel rotations (in degrees) necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(toRadians((BASE_WIDTH / 2) * angle));
  }
  
  /**
   * Sets the speed of both motors to the same values.
   * 
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    setSpeeds(speed, speed);
  }
  
  /**
   * Sets the speed of both motors to different values.
   * 
   * @param leftSpeed the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }
  
  /**
   * Sets the acceleration of both motors.
   * 
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
  }
  
  /**
   * Turn to provided angle, uses current odometer theta value to calculate angle to turn by.
   * @param angle The angle to turn to.
   */
  public static void turnTo(double angle) {

    Navigation.turnBy(minimalAngle(odometer.getTheta(), angle));

  }
  
}
