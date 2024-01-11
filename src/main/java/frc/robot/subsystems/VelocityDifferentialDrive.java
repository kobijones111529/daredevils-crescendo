package frc.robot.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface VelocityDifferentialDrive extends Subsystem {
	void velocityTankDrive(Measure<Velocity<Distance>> left, Measure<Velocity<Distance>> right);
	void velocityArcadeDrive(Measure<Velocity<Distance>> move, Measure<Velocity<Angle>> turn);
}
