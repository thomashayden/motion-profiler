package testing;

import profiler.*;

import java.util.ArrayList;

public class RudementaryTester {

  public static void main(String[] args) {
    ArrayList<ConstraintRange> points = new ArrayList<ConstraintRange>();
    points.add(new ConstraintRange(new Position(0), new Position(20), 10));
    points.add(new ConstraintRange(new Position(20), new Position(50), 16));
    points.add(new ConstraintRange(new Position(50), new Position(90), 14));
    TargetPositionPath path = new TargetPositionPath(points);
    Path1D p1d = path.convertToPath1D(3,4);
    for(Path1DPoint p : p1d.getPoints()) {
      System.out.println(p.toString());
    }
  }
}
