package frc.robot.subsystems.comp.drivetrain.capabilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public interface Odometry {
  DifferentialDriveOdometry getOdometry();
}
