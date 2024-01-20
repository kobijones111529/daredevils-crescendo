package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Intake extends Subsystem {
  class Actuate {
    public enum Position {
      Down,
      Up
    }

    public enum Status {
      Going,
      There
    }

    private Actuate() {
      throw new UnsupportedOperationException("Unsupported call to constructor of utility class");
    }
  }

  void run(double speed);

  void setPosition(Actuate.Position position);

  Actuate.Position getPosition();

  Actuate.Status getStatus();
}
