package frc.robot.subsystems.comp;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EncoderDifferentialDrive;
import frc.robot.subsystems.SimpleDifferentialDrive;

import java.util.List;
import java.util.stream.IntStream;

public class CompDrivetrain extends SubsystemBase implements SimpleDifferentialDrive, EncoderDifferentialDrive {

  public record Config(
    DriveGroup driveLeft,
    DriveGroup driveRight
  ) {
    public record DriveGroup(
      boolean inverted,
      int primaryID,
      IntStream backupIDs,
      Encoder encoder
    ) {
      public record Encoder(
        int channelA,
        int channelB,
        boolean reversed,
        Measure<Distance> distancePerPulse
      ) {}
    }
  }

  private sealed interface DriveControlMode {
    record Stop() implements DriveControlMode {}

    record Tank(double left, double right) implements DriveControlMode {}

    record Arcade(double move, double turn) implements DriveControlMode {}
  }

  private record DriveGroup(
    CANSparkMax primary,
    List<CANSparkMax> backups,
    Encoder encoder
  ) {}

  private record NetworkTableEntries(
    NetworkTableEntry controlMode
  ) {}

  private final NetworkTableEntries networkTableEntries;

  private final DriveGroup driveLeft;
  private final DriveGroup driveRight;

  private DriveControlMode driveControlMode = new DriveControlMode.Stop();

  public CompDrivetrain(Config config, NetworkTable networkTable) {
    networkTableEntries = new NetworkTableEntries(
      networkTable.getEntry("Control mode")
    );

    try (
      var leftPrimary = new CANSparkMax(config.driveLeft.primaryID, CANSparkLowLevel.MotorType.kBrushless);
      var rightPrimary = new CANSparkMax(config.driveRight.primaryID, CANSparkLowLevel.MotorType.kBrushless);
      var leftEncoder = new Encoder(
        config.driveLeft.encoder.channelA,
        config.driveLeft.encoder.channelB,
        config.driveLeft.encoder.reversed
      );
      var rightEncoder = new Encoder(
        config.driveRight.encoder.channelA,
        config.driveRight.encoder.channelB,
        config.driveRight.encoder.reversed
      )
    ) {
      leftPrimary.setInverted(config.driveLeft.inverted);
      rightPrimary.setInverted(config.driveRight.inverted);

      var leftBackups = config.driveLeft.backupIDs.mapToObj((id) -> createDriveBackup(leftPrimary, id)).toList();
      var rightBackups = config.driveRight.backupIDs.mapToObj((id) -> createDriveBackup(rightPrimary, id)).toList();

      leftEncoder.setDistancePerPulse(config.driveLeft.encoder.distancePerPulse.baseUnitMagnitude());
      rightEncoder.setDistancePerPulse(config.driveRight.encoder.distancePerPulse.baseUnitMagnitude());

      driveLeft = new DriveGroup(leftPrimary, leftBackups, leftEncoder);
      driveRight = new DriveGroup(rightPrimary, rightBackups, rightEncoder);
    }
  }

  private CANSparkMax createDriveBackup(CANSparkMax primary, int id) {
    try (var backup = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)) {
      backup.follow(primary);
      return backup;
    }
  }

  @Override
  public void periodic() {
    logOutputs();

    switch (driveControlMode) {
      case DriveControlMode.Stop ignore -> {
        driveLeft.primary.set(0);
        driveRight.primary.set(0);
      }
      case DriveControlMode.Tank tank -> {
        var wheelSpeeds = DifferentialDrive.tankDriveIK(tank.left, tank.right, false);
        driveLeft.primary.set(wheelSpeeds.left);
        driveRight.primary.set(wheelSpeeds.right);
      }
      case DriveControlMode.Arcade arcade -> {
        var wheelSpeeds = DifferentialDrive.arcadeDriveIK(arcade.move, arcade.turn, false);
        driveLeft.primary.set(wheelSpeeds.left);
        driveRight.primary.set(wheelSpeeds.right);
      }
    }
  }

  private void logOutputs() {
    networkTableEntries.controlMode.setString(
      switch (driveControlMode) {
        case DriveControlMode.Stop ignore -> "Stop";
        case DriveControlMode.Tank tank -> String.format("Tank (%.2f, %.2f)", tank.left, tank.right);
        case DriveControlMode.Arcade arcade -> String.format("Arcade (%.2f, %.2f)", arcade.move, arcade.turn);
      }
    );
  }

  @Override
  public void stop() {
    driveControlMode = new DriveControlMode.Stop();
  }

  @Override
  public void tankDrive(double left, double right) {
    driveControlMode = new DriveControlMode.Tank(left, right);
  }

  @Override
  public void arcadeDrive(double move, double turn) {
    driveControlMode = new DriveControlMode.Arcade(move, turn);
  }

  @Override
  public Measure<Distance> getLeftDistance() {
    double inBaseUnits = driveLeft.encoder.getDistance();
    return BaseUnits.Distance.of(inBaseUnits);
  }

  @Override
  public Measure<Distance> getRightDistance() {
    double inBaseUnits = driveRight.encoder.getDistance();
    return BaseUnits.Distance.of(inBaseUnits);
  }

  @Override
  public Measure<Velocity<Distance>> getLeftVelocity() {
    double inBaseUnits = driveLeft.encoder.getRate();
    return BaseUnits.Velocity.of(inBaseUnits);
  }

  @Override
  public Measure<Velocity<Distance>> getRightVelocity() {
    double inBaseUnits = driveLeft.encoder.getRate();
    return BaseUnits.Velocity.of(inBaseUnits);
  }
}
