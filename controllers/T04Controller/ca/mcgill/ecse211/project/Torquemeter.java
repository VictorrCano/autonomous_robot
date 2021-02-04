package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;

public class Torquemeter implements Runnable {

  /** Boolean used outside the class to know if the thread is running. */
  public static boolean running = true; 
  public static Torquemeter torquem;   
  /** Arraylist that stores torque measurements. */
  public static ArrayList<Double> torqueSamplesArrayList;
  public static int index;
  /** New average variable for each block pushed. */
  public static double average;

  /**
   * Get a singleton of the torqumeter. 
   * @return a new instance or the current torqumeter if one exists
   */
  public static synchronized Torquemeter getTorquemeter() {
    if (torquem == null) {
      torquem = new Torquemeter();
    }
    return torquem;
  }

  /**
   * Set the running to false, if we want the thread to stop running.
   */
  public static void terminate() {
    running = false;
  }

  /**
   * Run the torqumeter thread.
   */
  public void run() {
    //if flagged to run
    while (running) {
      
      measureTorque();

      try {
        Thread.sleep(200);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Get the average torque of the values stored in the arraylist.
   * @return double : the result of the avergae calculation.
   */
  public static double getAverage() {
    double sum = 0;
    for (int i = 0; i < torqueSamplesArrayList.size(); i++) {
      
      sum += torqueSamplesArrayList.get(i);
    }
    return round(sum / torqueSamplesArrayList.size(), 4);
  }

  /**
   * Measure the torque and add the sample to the sample array.
   */
  public void measureTorque() {
    double leftTorque = leftMotor.getTorque() * 100;
    double rightTorque = rightMotor.getTorque() * 100;
    double average = (leftTorque + rightTorque) / 2;

    //If average is above threshold
    if (average > 20.0) {

      torqueSamplesArrayList.add(average);
    }
  }


  /**
   * Round long double or floating point values for display to a desired decimal place.
   * 
   * @param value : the value to be rounded
   * @param places : number of places to round value
   */
  public static double round(double value, int places) {
    
    //places can not be negative
    if (places < 0) {
      throw new IllegalArgumentException();
    }

    BigDecimal bd = BigDecimal.valueOf(value);
    bd = bd.setScale(places, RoundingMode.HALF_UP);
    return bd.doubleValue();
  }

  /**
   * Set the torqumeter to run. 
   */
  public static void initialize() {
    //set running to true, clear array and index
    running = true;
    index = 0;
    torqueSamplesArrayList = new ArrayList<Double>(100);
  }
  
  /**
  *Determine the weight of the block being pushed into bin.
  */ 
  public static void decideOnWeightOfBlock() {
    System.out.println("Torque is " + Torquemeter.getAverage());
    double [] weights = {200.0, 210.0, 225.0, 250.0}; //array containing the weight of the blocks
    if (Torquemeter.getAverage() < weights[0]) {
      System.out.println("Container with weight 0.5 kg is identified ");
    }
    else if (Torquemeter.getAverage() < weights[1]) {
      System.out.println("Container with weight 1.0 kg is identified ");
    }
    else if (Torquemeter.getAverage() < weights[2]) {
      System.out.println("Container with weight 2.0 kg is identified");
    }
    else {
      System.out.println("Container with weight 3.0 kg is identified ");
    }
    running = false;
  }
}
