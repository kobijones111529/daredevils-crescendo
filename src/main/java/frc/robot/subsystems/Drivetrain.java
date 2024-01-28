package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.comp.drivetrain.capabilities.*;

import java.util.Optional;

public interface Drivetrain extends Subsystem {
  SimpleDrive getSimpleDrive();
  Optional<VelocityDrive> getVelocityDrive();
  Optional<EncoderDistance> getEncoderDistance();
  Optional<EncoderVelocity> getEncoderVelocity();
  Optional<Gyro> getGyro();
  Optional<Odometry> getOdometry();
  Optional<Kinematics> getKinematics();
}
