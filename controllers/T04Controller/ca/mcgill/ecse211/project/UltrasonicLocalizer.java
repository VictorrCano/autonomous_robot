package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.INVALID_SAMPLE_LIMIT;
import static ca.mcgill.ecse211.project.Resources.MAX_SENSOR_DIST;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.usSensor;
import static ca.mcgill.ecse211.project.Resources.*;

import java.math.BigDecimal;
import java.math.RoundingMode;


/**
 * Class to control the ultrasonic localization .
 */
public class UltrasonicLocalizer {
  
  private static int prevDist = 0;
  private static int newDist = 0;

  private static float[] usData = new float[usSensor.sampleSize()];
  
  private static double angleA = 0;
  private static double angleB = 0;
  private static double averageAngle = 0;
  private static double finalAngle = 0;
  /** Boolean used in Main to determine if US localization is in process.*/
  public static boolean inProcess = true;
  
  
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;
  
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  
  /**localize method for robot using UltraSonic sensor. */
  public static void localize() {
    Navigation.setSpeed(FAST_ROTATE_SPEED);
    
    //face away from walls before starting localization
    if (readUsDistance() < 100) {
      while (readUsDistance() < 100) {
        LightLocalizer.turnBy(15);
      }
    }
    
    //reset odometer theta count to 0 for more accurate angleA/angleB measurements
    odometer.setTheta(0);

    //find first angleA spinning clockwise using rising method
    while (rising() > 30) {
      LightLocalizer.turnBy(15);
    }
    
    //setting angle A using odometer
    angleA = odometer.getTheta(); //roughly
    
    //turn away from wall before begin search for angleB, in case 
    //robot went past 30cm mark on ultrasonic sensor
    LightLocalizer.turnBy(-30);
    
    //begin searching for angleB in counter clockwise direction
    while (readUsDistance() > 30) {

      LightLocalizer.turnBy(-15);

    }
    
    //setting angleB with odometer values
    angleB = odometer.getTheta(); //use odometer value
    
    //calculating average angle and maniupulating to obtain final angle
    averageAngle = (angleA - angleB) / 2;
    finalAngle = averageAngle + 135;
    LightLocalizer.turnBy(finalAngle);
    inProcess = false;
   
  }

  
  
  
  /**rising method to check angleA is an approaching distance. */
  public static int rising() {
    
    prevDist = newDist;
    
    newDist = readUsDistance();
    
    //if first time running rising, update pervDist
    if (prevDist == 0) {

      return 1000; 
    //if wall is approaching, return wall distance  
    } else if (prevDist > newDist) {

      return newDist;
    //if wall is distancing, return 1000 to avoid breaking while loop  
    } else {
      //newDist > prevDist so falling, return 1000

      return 1000;
    }

    
  }
  

  /** Returns the filtered distance between the US sensor and an obstacle in cm. */
  public static int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, cast to int, and filter
    return filter((int) (usData[0] * 100.0));
  }
  
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  static int filter(int distance) {
    if (distance >= MAX_SENSOR_DIST && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < MAX_SENSOR_DIST) {
        invalidSampleCount = 0; // reset filter and remember the input distance.
      }
      prevDistance = distance;
      return distance;
    }
  
  }
  
  /**
   * Round long double or floating point values for display to a desired decimal place.
   * 
   * @param value : the value to be rounded
   * @param places : number of places to round value
   */
  public static double round(double value, int places) {
    if (places < 0) {
      throw new IllegalArgumentException();
    }

    BigDecimal bd = BigDecimal.valueOf(value);
    bd = bd.setScale(places, RoundingMode.HALF_UP);
    return bd.doubleValue();
  }

}

