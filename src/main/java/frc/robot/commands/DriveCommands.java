package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.comp.drivetrain.capabilities.SimpleDrive;

import java.util.function.DoubleSupplier;

public class DriveCommands {
  public static Command tankDrive(Subsystem drivetrain, SimpleDrive simpleDrive, DoubleSupplier left, DoubleSupplier right) {
    return drivetrain.runEnd(
      () -> simpleDrive.tank(left.getAsDouble(), right.getAsDouble()),
      simpleDrive::stop
    );
  }

  public static Command arcadeDrive(Subsystem drivetrain, SimpleDrive simpleDrive, DoubleSupplier move, DoubleSupplier turn) {
    return drivetrain.runEnd(
      () -> simpleDrive.arcade(move.getAsDouble(), turn.getAsDouble()),
      simpleDrive::stop
    );
  }
  
  

  private DriveCommands() {
    throw new UnsupportedOperationException("Unsupported call to constructor of utility class");
  }
}
