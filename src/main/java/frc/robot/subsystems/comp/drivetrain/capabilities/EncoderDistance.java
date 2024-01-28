package frc.robot.subsystems.comp.drivetrain.capabilities;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public interface EncoderDistance {
  Measure<Distance> getLeft();
  Measure<Distance> getRight();
}
