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

public class MockDrivetrain extends SubsystemBase implements Drivetrain {

  private sealed interface DriveControlMode {
    record Stop() implements DriveControlMode {}

    record TankDrive(double left, double right) implements DriveControlMode {}

    record ArcadeDrive(double move, double turn) implements DriveControlMode {}
  }

  private sealed interface DriveOutput {
    record Stop() implements DriveOutput {}

    record Percent(double left, double right) implements DriveOutput {}
  }

  private final NetworkTable controlNetworkTable;
  private final NetworkTable outputNetworkTable;
  private DriveControlMode driveControlMode = new DriveControlMode.Stop();

  private final Measure<Distance> leftDistance = BaseUnits.Distance.zero();
  private final Measure<Distance> rightDistance = BaseUnits.Distance.zero();
  private final Measure<Velocity<Distance>> leftVelocity = BaseUnits.Velocity.zero();
  private final Measure<Velocity<Distance>> rightVelocity = BaseUnits.Velocity.zero();

  public MockDrivetrain(NetworkTable networkTable) {
    controlNetworkTable = networkTable.getSubTable("Control");
    outputNetworkTable = networkTable.getSubTable("Output");
  }

  @Override
  public void periodic() {
    logControlMode(driveControlMode);

    var driveOutput = switch (driveControlMode) {
      case DriveControlMode.Stop ignore -> new DriveOutput.Stop();
      case DriveControlMode.TankDrive output -> {
        var wheelSpeeds = DifferentialDrive.tankDriveIK(output.left, output.right, false);
        yield new DriveOutput.Percent(wheelSpeeds.left, wheelSpeeds.right);
      }
      case DriveControlMode.ArcadeDrive output -> {
        var wheelSpeeds = DifferentialDrive.arcadeDriveIK(output.move, output.turn, false);
        yield new DriveOutput.Percent(wheelSpeeds.left, wheelSpeeds.right);
      }
    };

    logDriveOutput(driveOutput);
  }

  private void logControlMode(DriveControlMode output) {
    double left = 0;
    double right = 0;
    double move = 0;
    double turn = 0;

    switch (output) {
      case DriveControlMode.Stop ignore -> {}
      case DriveControlMode.TankDrive out -> {
        left = out.left;
        right = out.right;
      }
      case DriveControlMode.ArcadeDrive out -> {
        move = out.move;
        turn = out.turn;
      }
    }

    controlNetworkTable.putValue("Left %", NetworkTableValue.makeDouble(left));
    controlNetworkTable.putValue("Right %", NetworkTableValue.makeDouble(right));
    controlNetworkTable.putValue("Move %", NetworkTableValue.makeDouble(move));
    controlNetworkTable.putValue("Turn %", NetworkTableValue.makeDouble(turn));
  }

  private void logDriveOutput(DriveOutput output) {
    double left = 0;
    double right = 0;

    switch (output) {
      case DriveOutput.Stop ignore -> {}
      case DriveOutput.Percent out -> {
        left = out.left;
        right = out.right;
      }
    }

    outputNetworkTable.putValue("Left %", NetworkTableValue.makeDouble(left));
    outputNetworkTable.putValue("Right %", NetworkTableValue.makeDouble(right));
  }

  @Override
  public void stop() {
    driveControlMode = new DriveControlMode.Stop();
  }

  @Override
  public void tankDrive(double left, double right) {
    driveControlMode = new DriveControlMode.TankDrive(left, right);
  }

  @Override
  public void arcadeDrive(double move, double turn) {
    driveControlMode = new DriveControlMode.ArcadeDrive(move, turn);
  }

  @Override
  public Measure<Distance> getLeftDistance() {
    return leftDistance;
  }

  @Override
  public Measure<Distance> getRightDistance() {
    return rightDistance;
  }

  @Override
  public Measure<Velocity<Distance>> getLeftVelocity() {
    return leftVelocity;
  }

  @Override
  public Measure<Velocity<Distance>> getRightVelocity() {
    return rightVelocity;
  }
}
