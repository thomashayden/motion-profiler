package testing;

import profiler.*;

import java.util.ArrayList;

public class RudementaryTester {

  public static void main(String[] args) {
    ArrayList<ConstraintRange> points = new ArrayList<ConstraintRange>();
    points.add(new ConstraintRange(new Position(0), new Position(10), 5));
    points.add(new ConstraintRange(new Position(10), new Position(30), 8));
    TargetPositionPath path = new TargetPositionPath(points);
    Path1D p1d = path.convertToPath1D(1,2);
    for(Path1DPoint p : p1d.getPoints()) {
      System.out.println(p.toString());
    }
  }
}
