package frc.robot.config;

import java.util.List;

public class CAN {
  public static final class Drivetrain {
    public static final int driveLeftPrimary = 1;
    public static final List<Integer> driveLeftBackups = List.of(2);
    public static final int driveRightPrimary = 3;
    public static final List<Integer> driveRightBackups = List.of(4);
    
    public static final int pigeon = 5;

    private Drivetrain() {}
  }

  private CAN() {}
}
