package profiler;

/*
TODO:
Add support for the inability for the path to reach max velocity
 */

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

  public double getMaxVelocity() {
    return this.maxVelocity;
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
    } else if(isTriangularPath(startVelocity, endVelocity, maxAcceleration, maxDeceleration)) {
      return generateTriangularPath(startVelocity, endVelocity, maxAcceleration, maxDeceleration,
              startTime);
    } else {
      System.err.println("None of the path options were chosen!");
      System.exit(1);
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
  public Path1D generateNoChangePath(double velocity, int startTime) {
    System.out.println("Generating No Change Path");
    Path1D p1d = new Path1D();
    p1d.addPoint(new Path1DPoint(startTime, velocity));
    int endTime = (int) (startTime + ((end.getX() - start.getX()) / velocity)) * 1000;
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
  public boolean isTrapezoidalPath(double startVelocity, double endVelocity, double
          maxAcceleration, double maxDeceleration) {
    if(startVelocity > this.maxVelocity || endVelocity > this.maxVelocity) {
      return false;
    }
    double distanceCoveredAccel = distanceCovered(startVelocity, maxAcceleration, (int) (
            (maxVelocity - startVelocity) / maxAcceleration) * 1000);
    double distanceCoveredDecel = distanceCovered(maxVelocity, -1 * maxDeceleration, (int)
            ((maxVelocity - endVelocity) / maxDeceleration) * 1000);
    System.out.println("alledged distanceCoveredAccel:"+distanceCoveredAccel);
    System.out.println("alledged distanceCoveredDecel:"+distanceCoveredDecel);
    double distanceAvailable = end.getX() - start.getX();
    return distanceCoveredAccel + distanceCoveredDecel < distanceAvailable;
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
  public Path1D generateTrapezoidalPath(double startVelocity, double endVelocity,
                                         double maxAcceleration, double maxDeceleration,
                                         int startTime) {
    System.out.println("Generating Trapezoidal Path");

    if(startVelocity > this.maxVelocity || endVelocity > this.maxVelocity) {
      System.err.println("startVelocity:"+startVelocity+",endVelocity:"+endVelocity+"," +
              "maxVelocity:"+maxVelocity);
      System.exit(1);
    }

    Path1D p1d = new Path1D();
    p1d.addPoint(new Path1DPoint(startTime, startVelocity));

    int timeToAccel = (int) (((this.maxVelocity - startVelocity) / maxAcceleration) * 1000);
    System.out.println("tta:"+timeToAccel);
    int peakStartTime = timeToAccel + startTime;
    p1d.addPoint(new Path1DPoint(peakStartTime, this.maxVelocity));

    int timeToDecelerate = (int) (((this.maxVelocity - endVelocity) / maxDeceleration) * 1000);
    double distanceCoveredAccel = distanceCovered(startVelocity, maxAcceleration, timeToAccel);
    double distanceCoveredDecel = distanceCovered(this.maxVelocity, -1 * maxDeceleration,
            timeToDecelerate);
    System.out.println("distanceCoveredAccel:"+distanceCoveredAccel);
    System.out.println("distanceCoveredDecel:"+distanceCoveredDecel);
    double remainingDistance = this.end.getX() - this.start.getX() - distanceCoveredAccel -
            distanceCoveredDecel;
    //System.out.println(remainingDistance);
    int timeAtPeak = (int) ((remainingDistance * this.maxVelocity) * 1000);
    System.out.println("tap:"+timeAtPeak);
    int peakEndTime = peakStartTime + timeAtPeak;
    p1d.addPoint(new Path1DPoint(peakEndTime, this.maxVelocity));

    System.out.println("ttd:"+timeToDecelerate);
    int endTime = peakEndTime + timeToDecelerate;
    p1d.addPoint(new Path1DPoint(endTime, endVelocity));

    return p1d;
  }

  /**
   * Checks if the constraints will lead to a triangular motion path. Distance units may be
   * anything self consistent, but time must be in seconds.
   *
   * @param startVelocity the velocity at the start of the range
   * @param endVelocity the velocity at the end of the range
   * @param maxAcceleration the fastest the robot can accelerate
   * @param maxDeceleration the fastest the robot can decelerate
   * @return if the motion path will be triangular
   */
  private boolean isTriangularPath(double startVelocity, double endVelocity, double
          maxAcceleration, double maxDeceleration) {
    if(startVelocity > endVelocity) {
      return distanceCovered(startVelocity, -1 * maxDeceleration, (int) (1000 * (startVelocity -
              endVelocity) / maxDeceleration)) <= end.getX() - start.getX();
    } else {
      return distanceCovered(startVelocity, maxAcceleration, (int) (1000 * (endVelocity -
              startVelocity) / maxAcceleration)) <= end.getX() - start.getX();
    }
  }

  /**
   * CURRENTLY NOT ACCURATE. SHOULD PRODUCE A SUB OPTIMAL PATH. Generates a motion path with a
   * triangular shape. Distance unites may be anything self consistent, but time must be in seconds.
   *
   * @param startVelocity the starting velocity of the range
   * @param endVelocity the ending velocity of the range
   * @param maxAcceleration the maximum acceleration of the robot
   * @param maxDeceleration the maximum deceleration of the robot
   * @param startTime the time at the beginning of the range
   * @return a motion path with the CURRENTLY SUBoptimal motion path and timestamps
   */
  private Path1D generateTriangularPath(double startVelocity, double endVelocity, double
          maxAcceleration, double maxDeceleration, int startTime) {
    System.out.println("WARNING: Triangular path generated");

    Path1D p1d = new Path1D();
    if(startVelocity > endVelocity) {
      p1d.addPoint(new Path1DPoint(startTime, startVelocity));
      int timeToDecelerate = (int) (1000 * (startVelocity - endVelocity) / maxDeceleration);
      p1d.addPoint(new Path1DPoint(startTime + timeToDecelerate, endVelocity));
    } else {
      p1d.addPoint(new Path1DPoint(startTime, startVelocity));
      int timeToAccelerate = (int) (1000 * (endVelocity - startVelocity) / maxAcceleration);
      p1d.addPoint(new Path1DPoint(startTime + timeToAccelerate, endVelocity));
    }
    return p1d;
  }

  /**
   * Calculates the total distance covered while the robot is under acceleration. Distance units
   * may be anything self consistent, but time must be in milliseconds.
   *
   * @param initialVelocity the velocity at the beginning of the movement
   * @param acceleration    the acceleration of the robot
   * @param time            the time under acceleration in milliseconds
   * @return the total distance covered
   */
  private double distanceCovered(double initialVelocity, double acceleration, int time) {
    return (initialVelocity * (time / 1000.0)) + (0.5 * acceleration * Math.pow(time / 1000.0, 2));
  }
}
