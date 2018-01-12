package profiler;

public class ConstraintRange {

  private Position start;
  private Position end;
  private double maxVelocity;

  /**
   * Constructs a new ConstraintRangeObject
   * @param start the start point of this constraint range
   * @param end the end point of this constraint range
   * @param maxVelocity the maximum velocity within this constraint range
   */
  public ConstraintRange(Position start, Position end, double maxVelocity) {
    this.start = start;
    this.end = end;
    this.maxVelocity = maxVelocity;
  }

  /**
   * Generates the optimal motion profile from this constraint range with the given end constraints
   * @param startVelocity the velocity at the beginning of the range
   * @param endVelocity the velocity at the end of the range
   */
  public Path1D generatePath1D(double startVelocity, double endVelocity, double maxAcceleration) {
    Path1D p1d = new Path1D();
    if(startVelocity == endVelocity && endVelocity == this.maxVelocity) {
      p1d.addPoint(new Path1DPoint(startVelocity));
    }
  }

}
