package testing;

public class MotorController {

  private double speed;
  private Encoder encoder;

  public MotorController(Encoder e) {
    this.encoder = e;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public double getSpeed() {
    return this.speed;
  }
}
