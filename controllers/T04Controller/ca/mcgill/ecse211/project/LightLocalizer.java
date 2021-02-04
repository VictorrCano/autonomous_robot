package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;


import simlejos.hardware.ev3.LocalEV3;

public class LightLocalizer {
  /** Buffer (array) to store color sensor samples. */
  static float[] sampleColorR;
  /** Buffer (array) to store US samples. */
  static float[] sampleColorL;
  

  /** Boolean used to determine if the localization is running. */
  static boolean running = true;
  /** Boolean used to determine if the left motor is running. */
  static boolean runningLeft;
  /** Boolean used to determine if the right motor is running. */
  static boolean runningRight;
  /** Counter used to determine amount of 90 degree turns. */
  static int rotations = 0;

  public static boolean inProcess = true;

  /**
   * Method to localize the robot from an orientation of 0
   * within the first tile to waypoint (1,1).
   */
  public static void localize() {

    // Intial fowards movement
    moveStraight();
    runningLeft = true;
    runningRight = true;
    // Indexes for left and right array of left and right color sensor samples
    int indexR = 0;
    int indexL = 0;
    sampleColorL = new float[1000];
    sampleColorR = new float[1000];

    while (running) {

      if (runningRight) {
        rightColorSensor.fetchSample(sampleColorR, indexR);
        stopRightMotor(indexR);
        indexR++;
      }
      if (runningLeft) {
        leftColorSensor.fetchSample(sampleColorL, indexL);
        stopLeftMotor(indexL);
        indexL++;
      }
      // Once both motors have stopped we want to turn 90 degrees
      if ((!runningLeft) && (!runningRight)) {
        runningLeft = true;
        runningRight = true;
        setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
        Drive.moveStraightFor(0.022);
        setSpeeds(FAST_ROTATE_SPEED, -FAST_ROTATE_SPEED);
        // First 90 degree-rotation will be positive (rotate clock-wise)
        if (rotations == 0) {
          turnBy(90);
          rotations++;
        } else if (rotations == 1) {
          // Second 90 degree-rotation will be negative (rotate counter clock-wise)
          Drive.moveStraightFor(0.009);

          turnBy(-90);
          Drive.moveStraightFor(-0.06);
          Drive.setSpeed(SLOW_SPEED);
          rotations++;
        } else if (rotations == 2) {
          Drive.moveStraightFor(0.024);

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
          break;
        }
        moveStraightSameSpeed();
      }
      // Sleep the entire while-loop for a better sampling rate
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    inProcess = false;
  }

  /**
   * Runs localization before tunnel, and resets odometer to more precise value.
   */
  public static void localizeBeforeTunnel() {

    // Intial fowards movement
    moveStraight();
    running = true;
    runningLeft = true;
    runningRight = true;
    sampleColorL = new float[1000];
    sampleColorR = new float[1000];
    // Indexes for left and right array of left and right color sensor samples
    int indexR2 = 0;
    int indexL2 = 0;

    while (running) {

      if (runningRight) {
        rightColorSensor.fetchSample(sampleColorR, indexR2);
        stopRightMotor(indexR2);
        indexR2++;
      }
      if (runningLeft) {
        leftColorSensor.fetchSample(sampleColorL, indexL2);
        stopLeftMotor(indexL2);
        indexL2++;
      }
      // Once both motors have stopped we want to turn 90 degrees
      if ((!runningLeft) && (!runningRight)) {
        Navigation.moveStraightFor(-0.297);
        break;
      }
      // Sleep the entire while-loop for a better sampling rate
      try {
        Thread.sleep(5);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }


  }


  /**
   * Stop right motor once color sensor detection goes from high to low (tile --> line).
   *
   * @param i : the current index of sampleColorR
   */
  public static void stopRightMotor(int i) {
    if (i >= 1) {
      float difference =  sampleColorR[i - 1] - sampleColorR[i];
      if (difference >= 80.0) {
        if (!runningLeft) {
          try {
            Thread.sleep(8);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        rightMotor.stop();
        runningRight = false;
      }
    }
  }
  
  /**
   * Poll for line on tile for localization.
   */
  public static void moveTillLine() {
    float[] left = new float[leftColorSensor.sampleSize()];
    float[] right = new float[rightColorSensor.sampleSize()];
    leftMotor.forward();
    rightMotor.forward();
    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      leftColorSensor.fetchSample(left, 0);
      rightColorSensor.fetchSample(right, 0);

      // If left wheel sees black line turn counter-clockwise
      if (left[0] < 80 || ObjectDetection.oneObstacle) {
        leftMotor.stop();

      }
      // If right wheel sees black line turn clockwise
      if (right[0] < 80 || ObjectDetection.oneObstacle) {
        rightMotor.stop();
      }
    }
  }

  /**
   * Stop left motor once color sensor detection goes from high to low (tile --> line).
   *
   * @param i : the current index of sampleColorL
   */
  public static void stopLeftMotor(int i) {
    if (i >= 1) {
      float difference = sampleColorL[i - 1] - sampleColorL[i];
      if (difference >= 80.0) {
        if (!runningRight) {
          try {
            Thread.sleep(8);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        leftMotor.stop();
        runningLeft = false;
      }
    }
  }

  /**
   * Turns the robot by a specified angle.
   *
   * @param angle : the angle by which to turn in degrees.
   */
  public static void turnBy(double angle) {
    int rotations =  convertAngle(angle);
    leftMotor.rotate(rotations, true);
    rightMotor.rotate(-rotations, false);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   *
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    double radians = angle * Math.PI / 180.0;
    double arcL = radians * (BASE_WIDTH / 2);

    return convertDistance(arcL);
  }

  /**
   * Moves the robot straight indefinitely.
   */
  public static void moveStraight() {
    setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }
  
  /**
   * Method to move robot straight ahead with same speed.
   */
  public static void moveStraightSameSpeed() {
    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   *
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
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
   * Moves the robot straight for the given distance.
   *
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {

    double totalDistance = distance * TILE_SIZE;
    int rotations = convertDistance(totalDistance);

    leftMotor.rotate(rotations, true);
    rightMotor.rotate(rotations, false);
  }

}
