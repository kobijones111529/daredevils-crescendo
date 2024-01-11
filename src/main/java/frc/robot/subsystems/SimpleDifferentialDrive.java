package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SimpleDifferentialDrive extends Subsystem {
	void stop();
	void tankDrive(double left, double right);
	void arcadeDrive(double move, double turn);
}
