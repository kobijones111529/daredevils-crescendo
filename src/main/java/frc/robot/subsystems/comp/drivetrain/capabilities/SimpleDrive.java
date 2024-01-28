package frc.robot.subsystems.comp.drivetrain.capabilities;

public interface SimpleDrive {
  void stop();
  void tank(double left, double right);
  void arcade(double move, double turn);
}
