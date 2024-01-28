package frc.robot.config;

import edu.wpi.first.units.Units;
import frc.robot.subsystems.comp.drivetrain.CompetitionDrivetrain;

import java.util.Optional;

public class Config {
  public static final class Subsystems {
    public enum Type {
      Mock, Comp
    }

    public static final Type drivetrain = Type.Comp;

    private Subsystems() {}
  }

  public sealed interface Drivetrain {
    record Mock() implements Drivetrain {}

    record Comp(CompetitionDrivetrain.Config config) implements Drivetrain {}
  }

  public static final Drivetrain drivetrain = switch (Subsystems.drivetrain) {
    case Comp -> {
      CompetitionDrivetrain.Config config = new CompetitionDrivetrain.Config(
        new CompetitionDrivetrain.Config.DriveGroup(
          false,
          CAN.Drivetrain.driveLeftPrimary,
          CAN.Drivetrain.driveLeftBackups,
          Optional.of(
            new CompetitionDrivetrain.Config.DriveGroup.Encoder(Units.Inches.of(6))
          ),
          Optional.empty()
        ),
        new CompetitionDrivetrain.Config.DriveGroup(
          false,
          CAN.Drivetrain.driveRightPrimary,
          CAN.Drivetrain.driveRightBackups,
          Optional.of(
            new CompetitionDrivetrain.Config.DriveGroup.Encoder(Units.Inches.of(6))
          ),
          Optional.empty()
        ),
        3,
        new CompetitionDrivetrain.Config.Gyro(CAN.Drivetrain.pigeon),
        Optional.empty()
      );
      yield new Drivetrain.Comp(config);
    }
    case Mock -> new Drivetrain.Mock();
  };

  private Config() {}
}
