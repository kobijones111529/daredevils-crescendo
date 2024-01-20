package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.config.Config;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.comp.CompDrivetrain;
import frc.robot.subsystems.mock.MockDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain =
      switch (Config.drivetrain) {
        case Config.Drivetrain.Comp comp ->
            new CompDrivetrain(
                comp.config(),
                NetworkTableInstance.getDefault().getTable("Drivetrain (Competition)"));
        case Config.Drivetrain.Mock ignore ->
            new MockDrivetrain(NetworkTableInstance.getDefault().getTable("Drivetrain (Mock)"));
      };

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  enum Auto {
    Auto1,
    Auto2
  }

  private final SendableChooser<Auto> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("Auto 1", Auto.Auto1);
    autoChooser.addOption("Auto 2", Auto.Auto2);
    SmartDashboard.putData("Auto", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drivetrain, driverController::getLeftY, driverController::getRightX));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return switch (autoChooser.getSelected()) {
      case Auto.Auto1, Auto.Auto2 -> new InstantCommand();
    };
  }
}
