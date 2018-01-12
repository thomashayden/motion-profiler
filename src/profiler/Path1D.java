package profiler;

import java.util.ArrayList;

public class Path1D {

  ArrayList<Path1DPoint> path;

  public Path1D() {
    path = new ArrayList<Path1DPoint>();
  }

  public Path1D(ArrayList<Path1DPoint> path) {
    this.path = path;
  }

  public void addPoint(Path1DPoint point) {
    path.add(point);
  }

}
