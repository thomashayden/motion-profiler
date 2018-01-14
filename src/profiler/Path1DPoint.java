package profiler;

public class Path1DPoint {

  private int time;
  private double velocity;

  public Path1DPoint(double velocity) {
    this.velocity = velocity;
  }

  public Path1DPoint(int time, double velocity) {
    this.time = time;
    this.velocity = velocity;
  }

  public int getTime() {
    return this.time;
  }

  public double getVelocity() {
    return this.velocity;
  }

  @Override
  public String toString() {
    return "time:"+time+",velocity:"+velocity;
  }

}
