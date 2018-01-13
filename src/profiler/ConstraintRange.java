package profiler;

public class ConstraintRange {

  private Position start;
  private Position end;
  private double maxVelocity;

  /**
   * Constructs a new ConstraintRangeObject
   *
   * @param start       the start point of this constraint range
   * @param end         the end point of this constraint range
   * @param maxVelocity the maximum velocity within this constraint range
   */
  public ConstraintRange(Position start, Position end, double maxVelocity) {
    this.start = start;
    this.end = end;
    this.maxVelocity = maxVelocity;
  }

  /**
   * Generates the optimal motion profile from this constraint range with the given end constraints
   *
   * @param startTime       the time at which this range starts
   * @param startVelocity   the velocity at the beginning of the range
   * @param endVelocity     the velocity at the end of the range
   * @param maxAcceleration the fastest the robot can speed up
   * @param maxDeceleration the fastest the robot can slow down
   * @return the optimal motion profile constructed from the given constraints
   */
  public Path1D generatePath1D(int startTime, double startVelocity, double endVelocity,
                               double maxAcceleration, double maxDeceleration) {
    if (startVelocity == endVelocity && endVelocity == this.maxVelocity) {
      return generateNoChangePath(startVelocity, startTime);
    } else if (isTrapezoidalPath(startVelocity, endVelocity, maxAcceleration, maxDeceleration)) {
      return generateTrapezoidalPath(startVelocity, endVelocity, maxAcceleration, maxDeceleration,
              startTime);
    } else {
      System.err.println("YOU STUPID!!! THIS WASN'T WHAT YOU WERE TESTING!!!");
      System.exit(0);
      return null;
    }
  }

  /**
   * Generates a motion path with no change. It is the same velocity throughout. Distance units
   * may be anything self consistent, but time must be in seconds.
   *
   * @param velocity the velocity the robot is traveling at
   * @return a motion path with constant velocity and timestamps
   */
  private Path1D generateNoChangePath(double velocity, int startTime) {
    Path1D p1d = new Path1D();
    p1d.addPoint(new Path1DPoint(startTime, velocity));
    int endTime = (int) (startTime + (velocity * (end.getX() - start.getX()))) * 1000;
    p1d.addPoint(new Path1DPoint(endTime, velocity));
    return p1d;
  }

  /**
   * Checks if the constraints will lead to a trapezoidal motion path. Distance units
   * may be anything self consistent, but time must be in seconds.
   *
   * @param startVelocity   the velocity at the start of the range
   * @param endVelocity     the velocity at the end of the range
   * @param maxAcceleration the fastest the robot can accelerate
   * @param maxDeceleration the fastest the robot can decelerate
   * @return if the motion path will be trapezoidal
   */
  private boolean isTrapezoidalPath(double startVelocity, double endVelocity, double
          maxAcceleration, double maxDeceleration) {
    return distanceCovered(startVelocity, maxAcceleration, (int) ((maxVelocity - startVelocity) /
            maxAcceleration) / 1000) + distanceCovered(maxVelocity, -1 * maxDeceleration, (int)
            ((maxVelocity - endVelocity) / maxAcceleration) / 1000)
            < end.getX() - start.getX();
  }

  /**
   * Generates a motion path with a trapezoidal shape. Distance units
   * may be anything self consistent, but time must be in seconds.
   *
   * @param startVelocity   the velocity at the start of the path
   * @param endVelocity     the velocity at the end of the path
   * @param maxAcceleration the fastest the robot can accelerate
   * @param maxDeceleration the fastest the robot can decelerate
   * @param startTime       the time at the start of the path
   * @return a motion path with the optimal trapezoidal motion path and timestamps
   */
  private Path1D generateTrapezoidalPath(double startVelocity, double endVelocity,
                                         double maxAcceleration, double maxDeceleration,
                                         int startTime) {
    Path1D p1d = new Path1D();
    p1d.addPoint(new Path1DPoint(startTime, startVelocity));

    int timeToAccel = (int) ((this.maxVelocity - startVelocity) / maxAcceleration) / 1000;
    int peakStartTime = timeToAccel + startTime;
    p1d.addPoint(new Path1DPoint(peakStartTime, this.maxVelocity));

    int timeToDecelerate = (int) ((this.maxVelocity - endVelocity) / maxDeceleration) / 1000;
    double distanceCoveredAccel = distanceCovered(startVelocity, maxAcceleration, timeToAccel);
    double distanceCoveredDecel = distanceCovered(this.maxVelocity, -1 * maxDeceleration,
            timeToDecelerate);
    double remainingDistance = this.end.getX() - this.start.getX() - distanceCoveredAccel -
            distanceCoveredDecel;
    int timeAtPeak = (int) (remainingDistance * this.maxVelocity);
    int peakEndTime = peakStartTime + timeAtPeak;
    p1d.addPoint(new Path1DPoint(peakEndTime, this.maxVelocity));

    int endTime = peakEndTime + timeToDecelerate;
    p1d.addPoint(new Path1DPoint(endTime, endVelocity));

    return p1d;
  }

  private Path1D generateTriangularPath(double startVelocity, double endVelocity, double
          maxAcceleration, double maxDeceleration, int startTime) {
    Path1D p1d = new Path1D();
    p1d.addPoint(new Path1DPoint(startTime, startVelocity));

    p1d.addPoint(new Path1DPoint(peakTime, peakVelocity));

    // startEq = startVelocity + maxAcceleration*x
    // endEq   = endVelocity + -maxDecceleration*x +
    // startVelocity - endVelocity = -maxDecceleration - maxAcceleration
    // (startVelocity - endVelocity) / (-maxDecceleration - maxAcceleration) = x

    p1d.addPoint(new Path1DPoint(endTime, endVelocity));
  }

  /**
   * Calculates the total distance covered while the robot is under acceleration. Distance units
   * may be anything self consistent, but time must be in seconds.
   *
   * @param initialVelocity the velocity at the beginning of the movement
   * @param acceleration    the acceleration of the robot
   * @param time            the time under acceleration
   * @return the total distance covered
   */
  private double distanceCovered(double initialVelocity, double acceleration, int time) {
    return (initialVelocity * (time / 1000.0)) + (0.5 * acceleration * Math.pow(time / 1000.0, 2));
  }
}
