package frc.robot.config;

import edu.wpi.first.units.Units;
import frc.robot.subsystems.comp.CompDrivetrain;

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
    record Comp(CompDrivetrain.Config config) implements Drivetrain {}
  }

  public static final Drivetrain drivetrain = switch (Subsystems.drivetrain) {
    case Comp -> {
      CompDrivetrain.Config config = new CompDrivetrain.Config(
        1,
        new CompDrivetrain.Config.DriveGroup(
          false,
          CAN.Drivetrain.driveLeftPrimary,
          CAN.Drivetrain.driveLeftBackups,
          new CompDrivetrain.Config.DriveGroup.Encoder(
            DIO.Drivetrain.leftEncoderChannelA,
            DIO.Drivetrain.leftEncoderChannelB,
            false,
            Units.Inches.one()
          )
        ),
        new CompDrivetrain.Config.DriveGroup(
          false,
          CAN.Drivetrain.driveRightPrimary,
          CAN.Drivetrain.driveRightBackups,
          new CompDrivetrain.Config.DriveGroup.Encoder(
            DIO.Drivetrain.rightEncoderChannelA,
            DIO.Drivetrain.rightEncoderChannelB,
            false,
            Units.Inches.one()
          )
        )
      );
      yield new Drivetrain.Comp(config);
    }
    case Mock -> new Drivetrain.Mock();
  };

  private Config() {}
}
