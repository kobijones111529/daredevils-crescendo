package frc.robot.subsystems;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface EncoderDifferentialDrive extends Subsystem {
	Measure<Distance> getLeftDistance();
	Measure<Distance> getRightDistance();
	Measure<Velocity<Distance>> getLeftVelocity();
	Measure<Velocity<Distance>> getRightVelocity();
}
