package frc.robot.subsystems.comp.drivetrain.capabilities;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface EncoderVelocity {
  Measure<Velocity<Distance>> getLeft();
  Measure<Velocity<Distance>> getRight();
}
