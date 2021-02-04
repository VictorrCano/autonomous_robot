package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.BASE_WIDTH;
import static ca.mcgill.ecse211.project.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

/**
 * Class that contains methods to Drive the robot.
 */
public class Drive {

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in meters, may be negative
   */
  public static void moveStraightFor(double distance) {
    // calculating total number of motor rotations according to set SQUARE_LENGTH
    // Note, set the booleans to continue executing right motor rotation while left
    // motor turns
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
    
  }
  
  /**
   * Moves the robot straight for the given distance with a small correction
   * to account for the inaccuracy of the turns the robot makes.
   * @param distance in meters, may be negative
   * @param correction the angle at which the robot needs to correct
   *     in degrees, may be negative
   */
  public static void moveStraightFor(double distance, int correction) {
    // calculating total number of motor rotations according to set SQUARE_LENGTH
    // Note, set the booleans to continue executing right motor rotation while left
    // motor turns
    setSpeed(FORWARD_SPEED);

    //if the distance is too short for a correction
    if (distance < TILE_SIZE) {
      leftMotor.rotate(convertDistance(distance), true);
      rightMotor.rotate(convertDistance(distance), false);
    } else {
      //correct every two tiles by the specified angle
      int numberTiles = (int) (distance / (2 * TILE_SIZE));
      for (int i = 0; i < numberTiles; i++) {
        leftMotor.rotate(convertDistance(2 * TILE_SIZE), true);
        rightMotor.rotate(convertDistance(2 * TILE_SIZE), false);
        turnBy(correction);
          
      }
      //Travel any reaming distance not covered in the loop
      double remainingDistance = distance - (numberTiles * 2 * TILE_SIZE);
      leftMotor.rotate(convertDistance(remainingDistance), true);
      rightMotor.rotate(convertDistance(remainingDistance), false);
      
    }
    
  }
  
  
  /**
   * Moves the robot backwards for a distance of 14 cm.
   * 
   */
  public static void moveBack() {
    // calculating total number of motor rotations according to distance needed
    // Note, set the booleans to continue executing right motor rotation while left
    // motor turns
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(-convertDistance(0.14), true);
    rightMotor.rotate(-convertDistance(0.14), false);

  }

  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees,
   * calling {@code turnBy(90)} will make the robot turn to 180 degrees, but
   * calling {@code Navigation.turnTo(90)} should do nothing (since the robot is
   * already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    // executing right hand turns
    // Set the booleans to continue executing right motor rotation while left motor
    // turns
    // This only turns right, as such the right motor moves backwards and the left
    // moves forward
    
    setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-1 * convertAngle(angle), false);
    setSpeed(FORWARD_SPEED);
    
  }
  

  /**
   * Converts input distance to the total rotation of each wheel needed to cover
   * that distance.
   * 
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    // converting distance required to travel into motor rotations
    return (int) ((distance / WHEEL_RAD) * (180 / Math.PI));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the
   * robot by that angle.
   * 
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    // convert degree angle input into motor rotations
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }

  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
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
   * @param leftSpeed  the speed of the left motor in degrees per second
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
}
