package frc.robot.subsystems.comp.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.comp.drivetrain.capabilities.*;

import java.text.MessageFormat;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

public class CompetitionDrivetrain extends SubsystemBase implements Drivetrain {

  public record Config(
    DriveGroup driveLeft,
    DriveGroup driveRight,
    double driveRateLimit,
    Optional<Gyro> gyro,
    Optional<Measure<Distance>> trackWidth
  ) {
    public record DriveGroup(
      boolean inverted,
      int primaryID,
      List<Integer> backupIDs,
      Optional<Encoder> encoder,
      Optional<SimpleMotorFeedforward> feedforward
    ) {
      public record Encoder(Measure<Distance> distancePerEncoderUnit) {}
    }

    public record Gyro(int id) {}
  }

  private record DriveGroup(CANSparkMax primary, List<CANSparkMax> backups, SlewRateLimiter rateLimiter) {}

  private enum ControlMode {
    Stop, Tank, Arcade
  }

  private record NetworkTableEntries(
    NetworkTableEntry controlMode,
    NetworkTableEntry leftOutput,
    NetworkTableEntry rightOutput,
    NetworkTableEntry leftPrimaryAppliedOutput,
    List<NetworkTableEntry> leftBackupAppliedOutput,
    NetworkTableEntry rightPrimaryAppliedOutput,
    List<NetworkTableEntry> rightBackupAppliedOutput,
    NetworkTableEntry leftPrimaryOutputCurrent,
    List<NetworkTableEntry> leftBackupOutputCurrent,
    NetworkTableEntry rightPrimaryOutputCurrent,
    List<NetworkTableEntry> rightBackupOutputCurrent,
    NetworkTableEntry leftTargetVelocity,
    NetworkTableEntry rightTargetVelocity,
    NetworkTableEntry leftVelocity,
    NetworkTableEntry rightVelocity
  ) {}

  private final NetworkTableEntries networkTableEntries;

  private final DriveGroup driveLeft;
  private final DriveGroup driveRight;

  private final Optional<Pigeon2> pigeon;

  private final Optional<DifferentialDriveOdometry> differentialOdometry;
  private final Optional<DifferentialDriveKinematics> differentialKinematics;

  private Pair<ControlMode, Runnable> driveOutput = new Pair<>(ControlMode.Stop, () -> {});

  public CompetitionDrivetrain(Config config, NetworkTable networkTable) {
    networkTableEntries = new NetworkTableEntries(
      networkTable.getEntry("Control mode"),
      networkTable.getEntry("Output (left)"),
      networkTable.getEntry("Output (right)"),
      networkTable.getEntry(
        MessageFormat.format(
          "Applied output (left | primary | id {0})",
          config.driveLeft.primaryID
        )
      ),
      config.driveLeft.backupIDs.stream().map(id ->
        networkTable.getEntry(
          MessageFormat.format("Applied output (left | backup | id {0})", id)
        )
      ).toList(),
      networkTable.getEntry(
        MessageFormat.format(
          "Applied output (right | primary | id {0})",
          config.driveRight.primaryID
        )
      ),
      config.driveRight.backupIDs.stream().map(id ->
        networkTable.getEntry(
          MessageFormat.format("Applied output (right | backup | id {0})", id)
        )
      ).toList(),
      networkTable.getEntry("Output current (left | primary | amps)"),
      config.driveLeft.backupIDs.stream().map(id ->
        networkTable.getEntry(
          MessageFormat.format("Output current (left | backup | id {0} | amps", id)
        )
      ).toList(),
      networkTable.getEntry("Output current (right | primary | amps)"),
      config.driveRight.backupIDs.stream().map(id ->
        networkTable.getEntry(
          MessageFormat.format("Output current (right | backup | id {0} | amps)", id)
        )
      ).toList(),
      networkTable.getEntry("Target velocity (left | m/s)"),
      networkTable.getEntry("Target velocity (right | m/s)"),
      networkTable.getEntry("Velocity (left | m/s)"),
      networkTable.getEntry("Velocity (right | m/s)")
    );

    var leftPrimary = new CANSparkMax(config.driveLeft.primaryID, CANSparkLowLevel.MotorType.kBrushless);
    var rightPrimary = new CANSparkMax(config.driveRight.primaryID, CANSparkLowLevel.MotorType.kBrushless);

    leftPrimary.setInverted(config.driveLeft.inverted);
    rightPrimary.setInverted(config.driveRight.inverted);

    var leftBackups = config.driveLeft.backupIDs.stream().map((id) -> createDriveBackup(leftPrimary, id)).toList();
    var rightBackups = config.driveRight.backupIDs.stream().map((id) -> createDriveBackup(rightPrimary, id)).toList();

    driveLeft = new DriveGroup(leftPrimary, leftBackups, new SlewRateLimiter(config.driveRateLimit));
    driveRight = new DriveGroup(rightPrimary, rightBackups, new SlewRateLimiter(config.driveRateLimit));

    pigeon = config.gyro.map(gyro -> new Pigeon2(gyro.id));

    velocityDrive = config.driveLeft.feedforward.flatMap(leftFeedforward ->
      config.driveRight.feedforward.map(rightFeedforward ->
        new VelocityDrive() {
          @Override
          public void tank(Measure<Velocity<Distance>> left, Measure<Velocity<Distance>> right) {
            double leftOut = leftFeedforward.calculate(left.in(Units.MetersPerSecond));
            double rightOut = rightFeedforward.calculate(right.in(Units.MetersPerSecond));
            driveLeft.primary.set(leftOut);
            driveRight.primary.set(rightOut);
          }

          @Override
          public void arcade(Measure<Velocity<Distance>> move, Measure<Velocity<Angle>> turn) {

          }
        }
      )
    );

    encoderDistance = config.driveLeft.encoder.flatMap(left ->
      config.driveRight.encoder.map(right ->
        new EncoderDistance() {
          @Override
          public Measure<Distance> getLeft() {
            return left.distancePerEncoderUnit.times(driveLeft.primary.getEncoder().getPosition());
          }

          @Override
          public Measure<Distance> getRight() {
            return right.distancePerEncoderUnit.times(driveRight.primary.getEncoder().getPosition());
          }
        }
      )
    );

    encoderVelocity = config.driveLeft.encoder.flatMap(left ->
      config.driveRight.encoder.map(
        right -> new EncoderVelocity() {
          @Override
          public Measure<Velocity<Distance>> getLeft() {
            return left.distancePerEncoderUnit.per(Units.Seconds.of(driveLeft.primary.getEncoder().getVelocity()));
          }

          @Override
          public Measure<Velocity<Distance>> getRight() {
            return right.distancePerEncoderUnit.per(Units.Seconds.of(driveRight.primary.getEncoder().getVelocity()));
          }
        }
      )
    );

    gyro = pigeon.map(pigeon ->
      new Gyro() {
        @Override
        public Measure<Angle> getAngle() {
          return Units.Degrees.of(pigeon.getAngle());
        }

        @Override
        public Rotation2d getRotation2d() {
          return pigeon.getRotation2d();
        }
      }
    );

    differentialOdometry = encoderDistance.flatMap(encoderDistance -> gyro.map(gyro -> new DifferentialDriveOdometry(
      gyro.getRotation2d(),
      encoderDistance.getLeft(),
      encoderDistance.getRight()
    )));

    odometry = differentialOdometry.map(odometry -> () -> odometry);

    differentialKinematics = config.trackWidth.map(DifferentialDriveKinematics::new);

    kinematics = differentialKinematics.map(kinematics -> () -> kinematics);
  }

  private CANSparkMax createDriveBackup(CANSparkMax primary, int id) {
    var backup = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);

    backup.restoreFactoryDefaults();
    backup.follow(primary);

    return backup;
  }

  @Override
  public void periodic() {
    differentialOdometry.flatMap(odometry -> encoderDistance.flatMap(encoderDistance -> gyro.map(gyro ->
      odometry.update(
        gyro.getRotation2d(),
        encoderDistance.getLeft().in(Units.Meters),
        encoderDistance.getRight().in(Units.Meters)
      )
    )));

    driveOutput.getSecond().run();

    logOutputs();
  }

  private void logOutputs() {
    networkTableEntries.controlMode.setString(driveOutput.getFirst().toString());
    networkTableEntries.leftPrimaryAppliedOutput.setDouble(driveLeft.primary.getAppliedOutput());
    IntStream
      .range(0, Math.min(driveLeft.backups.size(), networkTableEntries.leftBackupAppliedOutput.size()))
      .forEach(i ->
        networkTableEntries.leftBackupAppliedOutput.get(i).setDouble(driveLeft.backups.get(i).getAppliedOutput())
      );
    networkTableEntries.rightPrimaryAppliedOutput.setDouble(driveRight.primary.getAppliedOutput());
    IntStream
      .range(0, Math.min(driveRight.backups.size(), networkTableEntries.rightBackupAppliedOutput.size()))
      .forEach(i ->
        networkTableEntries.rightBackupAppliedOutput.get(i).setDouble(driveRight.backups.get(i).getAppliedOutput())
      );
    networkTableEntries.leftPrimaryOutputCurrent.setDouble(driveLeft.primary.getOutputCurrent());
    IntStream
      .range(0, Math.min(driveLeft.backups.size(), networkTableEntries.leftBackupOutputCurrent.size()))
      .forEach(i ->
        networkTableEntries.leftBackupOutputCurrent.get(i).setDouble(driveLeft.backups.get(i).getOutputCurrent())
      );
    networkTableEntries.rightPrimaryOutputCurrent.setDouble(driveRight.primary.getOutputCurrent());
    IntStream
      .range(0, Math.min(driveRight.backups.size(), networkTableEntries.rightBackupOutputCurrent.size()))
      .forEach(i ->
        networkTableEntries.rightBackupOutputCurrent.get(i).setDouble(driveRight.backups.get(i).getOutputCurrent())
      );
  }

  private final SimpleDrive simpleDrive = new SimpleDrive() {
    @Override
    public void stop() {
      driveOutput = new Pair<>(ControlMode.Stop, () -> {
        driveLeft.primary.disable();
        driveRight.primary.disable();

        networkTableEntries.leftOutput.setDouble(0);
        networkTableEntries.rightOutput.setDouble(0);
      });
    }

    @Override
    public void tank(double left, double right) {
      driveOutput = new Pair<>(ControlMode.Tank, () -> {
        double leftOut = driveLeft.rateLimiter.calculate(left);
        double rightOut = driveRight.rateLimiter.calculate(right);
        driveLeft.primary.set(leftOut);
        driveRight.primary.set(rightOut);

        networkTableEntries.leftOutput.setDouble(leftOut);
        networkTableEntries.rightOutput.setDouble(rightOut);
      });
    }

    @Override
    public void arcade(double move, double turn) {
      driveOutput = new Pair<>(ControlMode.Arcade, () -> {
        WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(move, turn, false);
        double leftOut = driveLeft.rateLimiter.calculate(wheelSpeeds.left);
        double rightOut = driveRight.rateLimiter.calculate(wheelSpeeds.right);
        driveLeft.primary.set(leftOut);
        driveRight.primary.set(rightOut);

        networkTableEntries.leftOutput.setDouble(leftOut);
        networkTableEntries.rightOutput.setDouble(rightOut);
      });
    }
  };

  private final Optional<VelocityDrive> velocityDrive;

  private final Optional<EncoderDistance> encoderDistance;

  private final Optional<EncoderVelocity> encoderVelocity;

  private final Optional<Gyro> gyro;

  private final Optional<Odometry> odometry;
  private final Optional<Kinematics> kinematics;

  @Override
  public SimpleDrive getSimpleDrive() {
    return simpleDrive;
  }

  @Override
  public Optional<VelocityDrive> getVelocityDrive() {
    return velocityDrive;
  }

  @Override
  public Optional<EncoderDistance> getEncoderDistance() {
    return encoderDistance;
  }

  @Override
  public Optional<EncoderVelocity> getEncoderVelocity() {
    return encoderVelocity;
  }

  @Override
  public Optional<Gyro> getGyro() {
    return gyro;
  }

  @Override
  public Optional<Odometry> getOdometry() {
    return odometry;
  }

  @Override
  public Optional<Kinematics> getKinematics() {
    return kinematics;
  }
}
