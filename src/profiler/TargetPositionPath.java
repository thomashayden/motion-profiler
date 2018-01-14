package profiler;

import java.util.ArrayList;

public class TargetPositionPath {

  // The constraints along the desired path.
  private ArrayList<ConstraintRange> path;

  /**
   * Constructs a new TargetPositionPath object
   * @param path the constraing path for the robot to follow
   */
  public TargetPositionPath(ArrayList<ConstraintRange> path) {
    this.path = path;
  }

  public Path1D debugConvertToPath1D(double maxAcceleration, double maxDeceleration) {
    Path1D p1d = new Path1D();
    int startTime = 0;
    double startVelocity = 0;
    for(int i = 0; i < this.path.size(); i++) {
      double endVelocity;
      if(i == this.path.size()-1) {
        endVelocity = 0;
      } else {
        endVelocity = Math.min(this.path.get(i + 1).getMaxVelocity(),this.path.get(i).getMaxVelocity());
      }
      Path1D partPath1D = this.path.get(i).generatePath1D(startTime, startVelocity, endVelocity,
              maxAcceleration, maxDeceleration);
      for (int j = 0; j < partPath1D.getPoints().size(); j++) {
        p1d.addPoint(partPath1D.getPoints().get(j));
      }
      startTime = partPath1D.getPoints().get(i).getTime();
      startVelocity = partPath1D.getPoints().get(i).getVelocity();
    }
    return p1d;
  }

  public Path1D convertToPath1D(double maxAcceleration, double maxDeceleration) {
    Path1D p1d = new Path1D();
    int startTime = 0;
    double startVelocity = 0;
    for(int i = 0; i < this.path.size(); i++) {
      double endVelocity;
      if(i == this.path.size()-1) {
        endVelocity = 0;
      } else {
        endVelocity = Math.min(this.path.get(i + 1).getMaxVelocity(),this.path.get(i).getMaxVelocity());
      }
      Path1D partPath1D = this.path.get(i).generatePath1D(startTime, startVelocity, endVelocity,
              maxAcceleration, maxDeceleration);
      for (int j = 0; j < partPath1D.getPoints().size(); j++) {
        p1d.addPoint(partPath1D.getPoints().get(j));
      }
      startTime = partPath1D.getPoints().get(partPath1D.getPoints().size()-1).getTime();
      startVelocity = partPath1D.getPoints().get(partPath1D.getPoints().size()-1).getVelocity();
      System.out.println("start time:"+startTime);
      System.out.println("start velocity:"+startVelocity);
    }
    return p1d;
  }
}
