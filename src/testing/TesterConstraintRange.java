package testing;

import profiler.ConstraintRange;
import profiler.Path1DPoint;
import profiler.Position;

public class TesterConstraintRange {

  public static void main(String[] args) {
    testGenerateTrapezoidalPath();
  }

  private static void testGenerateNoChangePath() {
    ConstraintRange cr = new ConstraintRange(new Position(0), new Position(10), 5);
    for(Path1DPoint p : cr.generateNoChangePath(5, 0).getPoints()) {
      System.out.println(p.toString());
    }
  }

  private static void testGenerateTrapezoidalPath() {
    ConstraintRange cr = new ConstraintRange(new Position(10), new Position(100), 8);
    for(Path1DPoint p : cr.generateTrapezoidalPath(0, 0, 3, 3, 0).getPoints()) {
      System.out.println(p.toString());
    }
  }
}
