package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimpleDifferentialDrive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  public static Command tankDrive(
      SimpleDifferentialDrive drivetrain, DoubleSupplier left, DoubleSupplier right) {
    return drivetrain.runEnd(
        () -> drivetrain.tankDrive(left.getAsDouble(), right.getAsDouble()), drivetrain::stop);
  }

  public static Command arcadeDrive(
      SimpleDifferentialDrive drivetrain, DoubleSupplier move, DoubleSupplier turn) {
    return drivetrain.runEnd(
        () -> drivetrain.arcadeDrive(move.getAsDouble(), turn.getAsDouble()), drivetrain::stop);
  }

  private DriveCommands() {
    throw new UnsupportedOperationException("Unsupported call to constructor of utility class");
  }
}
