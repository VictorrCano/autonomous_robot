package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import java.lang.Thread;
import java.util.ArrayList;
import java.util.Collections;
import ca.mcgill.ecse211.playingfield.Point;
import simlejos.hardware.ev3.LocalEV3;


/**
 * Main class of the program.
 * We begin by starting an odometer thread to keep track of the robot's x-position, y-position,
 * and heading. Program proceeds to localize to the nearest grid point using ultrasonic
 * and light sensors. Once this is achieved, the robot travels to the center point of the 
 * tile immediately before the tunnel and  crosses it. In the search zone the robot
 * begins it's navigation, avoiding obstacles and detecting blocks using color sensor.
 */
public class Main {

  public static Point entryPoint;
  public static Point exitPoint;
  /**
   * The number of threads used in the program (main, odometer), 
   * other than the one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;

  /** Main entry point. */
  public static void main(String[] args) {
    initialize();
    new Thread(odometer).start();

    new Thread(() -> {
      UltrasonicLocalizer.localize();
      LightLocalizer.localize();
    }).start();
    while (LightLocalizer.inProcess || UltrasonicLocalizer.inProcess) {
      // Do nothing but delay the light localizaer so that it starts after the US localizer is done.
      waitUntilNextStep();
    } 


    Point lowerLeft = null;
    Point upperRight = null;
    // Reset odometer to match localization positioning.

    int corner = 0;
    if (redTeam == 04) {
      corner = redCorner;
      ObjectDetection.startingZone = red;
      setStartOdometer(red.ll, red.ur, corner);
      BlockMover.searchZone = szr;
      BlockMover.ramp = rr;
      lowerLeft = new Point(tnr.ll.x, tnr.ll.y);
      upperRight = new Point(tnr.ur.x, tnr.ur.y);

    } else if (greenTeam == 04) {
      ObjectDetection.startingZone = green;
      corner = greenCorner;
      setStartOdometer(green.ll, green.ur, corner);
      BlockMover.searchZone = szg;
      BlockMover.ramp = gr;
      lowerLeft = new Point(tng.ll.x, tng.ll.y);
      upperRight = new Point(tng.ur.x, tng.ur.y);
    }


    Point currentPoint = new Point(odometer.getXyt()[0] / TILE_SIZE,
        odometer.getXyt()[1] / TILE_SIZE);
    double angle = Navigation.getDestinationAngle(lowerLeft, upperRight);
    double tunnelDirection = 1;
    if ((angle < 60)) {
      tunnelDirection = -1;
    }
    entryPoint = new Point(lowerLeft.x - (tunnelDirection / 2),
        lowerLeft.y + (tunnelDirection / 2));
    exitPoint = new Point(upperRight.x + (tunnelDirection / 2),
        upperRight.y - (tunnelDirection / 2));
    if (Navigation.distanceBetween(currentPoint, entryPoint) 
        > Navigation.distanceBetween(currentPoint, exitPoint)) {
      Point temp = entryPoint;
      entryPoint = exitPoint;
      exitPoint = temp;
    }
   


    Point localizationPoint = new Point(currentPoint.x, currentPoint.y);
    new Thread(objectDetection).start();
    ObjectDetection.entryInitialize();
    Navigation.travelTo(exitPoint);


    odometer.setXyt(exitPoint.x * TILE_SIZE, exitPoint.y * TILE_SIZE, odometer.getTheta());
  
    OdometerCorrection.checkOdometerAfterCrossBridge(exitPoint);
    

    ObjectDetection.initalize();
    
 
    new Thread(objectDetection).start();

    ObjectDetection.resetToReturnHome(exitPoint);
    
    
    OdometerCorrection.checkOdometerToCrossBridge(exitPoint, entryPoint);
    
    Navigation.travelTo(entryPoint);
    if (corner == 1) {
      localizationPoint.x = localizationPoint.x + 0.6;
      localizationPoint.y = localizationPoint.y - 0.6;
    } else if (corner == 2) {
      localizationPoint.x = localizationPoint.x - 0.6;
      localizationPoint.y = localizationPoint.y - 0.6;
    } else if (corner == 3) {
      localizationPoint.x = localizationPoint.x - 0.6;
      localizationPoint.y = localizationPoint.y + 0.6;
    } else if (corner == 4) {
      localizationPoint.x = localizationPoint.x + 0.6;
      localizationPoint.y = localizationPoint.y + 0.6;
    }

    goNearCorner();
    Navigation.travelTo(localizationPoint);
    beepFiveTimes();
    


  }
  
  /**
   * Method that makes robot beep five times in Webots.
   */
  public static void beepFiveTimes() {
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
   * On way home, find initial corner to return to.
   */
  public static void goNearCorner() {
    Collections.reverse(ObjectDetection.startZonePath);
    for (Point point : ObjectDetection.startZonePath) {
      Navigation.travelTo(point);
    }
  }
  
  /**
   * Start odometer with passed coordinates.
   * @param ll  passed point
   * @param ur second passed point
   * @param corner  starting corner
   */
  private static void setStartOdometer(Point ll, Point ur, int corner) {

    if (corner % 3 == 0) {
      odometer.setX((TILE_SIZE * (ll.x + 1)));
    } else {
      odometer.setX((TILE_SIZE * (ur.x - 1)));
    }
    if (corner > 1) {
      odometer.setY((TILE_SIZE * (ur.y - 1)));
    } else {
      odometer.setY((TILE_SIZE * (ll.y + 1)));
    }
    odometer.setTheta((360 - 90 * corner) % 360);
  }

  /**
   * Example using WifiConnection to communicate with a server and receive 
   * data concerning the competition such as the
   * starting corner the robot is placed in.<br>
   *
   * <p>Keep in mind that this class is an <b>example</b> of how to use
   *  the Wi-Fi code; you must use the WifiConnection
   * class yourself in your own code as appropriate. In this example, we simply show
   *  how to get and process different
   * types of data.
   * <br>
   * 
   * <p>
   * There are two variables you MUST set manually (in Resources.java) before using this code:
   *
   * <ol>
   * <li>SERVER_IP: The IP address of the computer running the server applic
   * ation. This will be your own laptop, until
   * the beta beta demo or competition where this is the TA or professor's 
   * laptop. In that case, set the IP to the
   * default (indicated in Resources).</li>
   * <li>TEAM_NUMBER: your project team number.</li>
   * </ol>
   * <p>
   * Note: You can disable printing from the Wi-Fi code via ENABLE_DEBUG_WIFI_PRINT.
   *
   * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur, Olivier St-Martin Cormier
   */
  public static void wifiExample() {
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Red Team: " + redTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Red tunnel footprint, lower left y value: " + tnr.ll.y);

    // Example 3: Compare value
    if (szg.ll.x >= island.ll.x && szg.ll.y >= island.ll.y) {
      System.out.println("The green search zone is on the island.");
    } else {
      System.err.println("The green search zone is in the water!");
    }

    // Example 4: Calculate the area of a region
    System.out.println("The island area is " + island.getWidth() * island.getHeight() + ".");
  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps regularly.
   */
  private static void initialize() {
    // Run a few physics steps to make sure everything is initialized and has settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }

    // We are going to start two threads, so the total number of parties is 2
    setNumberOfParties(NUMBER_OF_THREADS);

    // Does not count as a thread because it is only for physics steps
    new Thread(() -> {
      while (performPhysicsStep()) {
        sleepFor(PHYSICS_STEP_PERIOD);
      }
    }).start();
  }

}
