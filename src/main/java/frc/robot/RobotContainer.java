package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.config.Config;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.comp.drivetrain.CompetitionDrivetrain;
import frc.robot.subsystems.mock.MockDrivetrain;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;

  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  enum Auto {Auto1, Auto2}

  private final SendableChooser<Auto> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Config.drivetrain instanceof Config.Drivetrain.Comp comp) {
      drivetrain = new CompetitionDrivetrain(
        comp.config(),
        NetworkTableInstance.getDefault().getTable("Drivetrain (competition)")
      );
    } else {
      drivetrain = new MockDrivetrain(NetworkTableInstance.getDefault().getTable("Drivetrain (mock)"));
    }

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
      DriveCommands.arcadeDrive(drivetrain, drivetrain.getSimpleDrive(), () -> -driverController.getLeftY(), driverController::getRightX)
    );

    drivetrain.getOdometry().flatMap(odometry ->
      drivetrain.getKinematics().map(kinematics -> {
          var command = new RamseteCommand(
            TrajectoryGenerator.generateTrajectory(
              List.of(),
              new TrajectoryConfig(
                Units.MetersPerSecond.of(1),
                Units.MetersPerSecondPerSecond.of(1)
              )
            ),
            () -> odometry.getOdometry().getPoseMeters(),
            new RamseteController(),
            kinematics.getKinematics(),
            (left, right) -> {},
            drivetrain
          );
          driverController.b().onTrue(command);
          return null;
        }
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return switch (autoChooser.getSelected()) {
      case Auto1, Auto2 -> new InstantCommand();
    };
  }
}

