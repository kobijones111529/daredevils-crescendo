package frc.robot.config;

import java.util.stream.IntStream;

public class CAN {
  public static final class Drivetrain {
    public static final int driveLeftPrimary = 1;
    public static final IntStream driveLeftBackups = IntStream.of(2);
    public static final int driveRightPrimary = 3;
    public static final IntStream driveRightBackups = IntStream.of(4);

    private Drivetrain() {}
  }

  private CAN() {}
}
