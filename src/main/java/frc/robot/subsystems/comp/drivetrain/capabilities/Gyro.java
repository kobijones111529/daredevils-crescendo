package frc.robot.subsystems.comp.drivetrain.capabilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public interface Gyro {
  Measure<Angle> getAngle();
  Rotation2d getRotation2d();
}
