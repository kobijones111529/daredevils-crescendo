package frc.robot.subsystems.comp.drivetrain.capabilities;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface VelocityDrive {
  void tank(Measure<Velocity<Distance>> left, Measure<Velocity<Distance>> right);
  void arcade(Measure<Velocity<Distance>> move, Measure<Velocity<Angle>> turn);
}
