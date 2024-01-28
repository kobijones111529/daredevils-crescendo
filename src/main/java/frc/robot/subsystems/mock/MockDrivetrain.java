package frc.robot.subsystems.mock;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EncoderDifferentialDrive;
import frc.robot.subsystems.SimpleDifferentialDrive;
import frc.robot.subsystems.comp.drivetrain.capabilities.*;

import java.util.Optional;

public class MockDrivetrain extends SubsystemBase implements Drivetrain {
  private final SimpleDrive simpleDrive = new SimpleDrive() {
    @Override
    public void stop() {
      
    }

    @Override
    public void tank(double left, double right) {

    }

    @Override
    public void arcade(double move, double turn) {

    }
  };
  
  public MockDrivetrain(NetworkTable networkTable) {
    
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public SimpleDrive getSimpleDrive() {
    return simpleDrive;
  }

  @Override
  public Optional<VelocityDrive> getVelocityDrive() {
    return Optional.empty();
  }

  @Override
  public Optional<EncoderDistance> getEncoderDistance() {
    return Optional.empty();
  }

  @Override
  public Optional<EncoderVelocity> getEncoderVelocity() {
    return Optional.empty();
  }

  @Override
  public Optional<Gyro> getGyro() {
    return Optional.empty();
  }

  @Override
  public Optional<Odometry> getOdometry() {
    return Optional.empty();
  }

  @Override
  public Optional<Kinematics> getKinematics() {
    return Optional.empty();
  }
}
