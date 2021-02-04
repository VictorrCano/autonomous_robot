package ca.mcgill.ecse211.playingfield;

import ca.mcgill.ecse211.project.Resources;

/** Line segment representing the approach of a ramp. */
public class RampEdge {

  /** The left endpoint of the ramp. */
  public Point left;
  
  /** The right endpoint of the ramp. */
  public Point right;
  public Region walledOffRegion;
  public Point blockDestination;
  public Point robotDestination1;
  public Point robotDestination2;
  
  /** Constructs a RampEdge. */
  public RampEdge(Point left, Point right) {
    this.left = left;
    this.right = right;
    double angle=Math.round(Math.atan2(left.y-right.y, left.x-right.x)*Resources.RAD_TO_DEGREE);
    System.out.println(angle);
    if(angle==270) {
      walledOffRegion=new Region(new Point(left.x-2,left.y),new Point(right.x, right.y));
      blockDestination=new Point(right.x+(1.0/3.0),right.y-(1.0/2.0));
      robotDestination1=new Point(right.x+1.3,right.y-(1.0/2.0));
      robotDestination2=new Point(right.x+1.2,right.y-(1.0/2.0));
      
    }
    if(angle==90) {
      walledOffRegion=new Region(new Point(right.x,right.y),new Point(left.x+2, left.y));
      blockDestination=new Point(right.x-(1.0/3.0),right.y+(1.0/2.0));
      robotDestination1=new Point(right.x-1.3,right.y+(1.0/2.0));
      robotDestination2=new Point(right.x-1.2,right.y+(1.0/2.0));
    }
    if(angle==0) {
      walledOffRegion=new Region(new Point(right.x,right.y-2),new Point(left.x,left.y));
      blockDestination=new Point(left.x-(1.0/2.0),left.y+(1.0/3.0));
      robotDestination1=new Point(left.x-(1.0/2.0),left.y+1.3);
      robotDestination2=new Point(left.x-(1.0/2.0),right.y+1.2);
    }
    if(angle==180) {
      walledOffRegion=new Region(new Point(left.x,left.y),new Point (right.x,right.y+2));
      blockDestination=new Point(left.x+(1.0/2.0),left.y-(1.0/3.0));
      robotDestination1=new Point(left.x+(1.0/2.0),left.y-1.3);
      robotDestination2=new Point(left.x+(1.0/2.0),right.y-1.2);
    }
  }
  @Override
  public String toString() {
    return ("Left Point:"+left+"   Right Point:"+right+"WalledOffRegion:"+walledOffRegion);
  }


}
