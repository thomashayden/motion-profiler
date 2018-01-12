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

  public Path1D convertToPath1D() {

  }

}
