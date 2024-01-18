package frc.robot.scala.robot

import edu.wpi.first.wpilibj.{RobotBase, Timer}
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.scala.robot.subsystems.comp.{Drivetrain, Intake}

import scala.math.*

class Container:
  val auto: Option[Command] = None

  private val drivetrain: Drivetrain = Drivetrain(config.drivetrain)
  private val intake: Intake = Intake(config.intake)

  private val xbox: CommandXboxController = CommandXboxController(
    config.driverStation.xbox
  )
  
  configureBindings()

  private def configureBindings(): Unit =
    // Drive
    drivetrain.setDefaultCommand(
      commands.drive.arcade(drivetrain, xbox.getLeftY, xbox.getRightX)
    )

    // Intake
    xbox.a
      .onTrue(
        commands.intake.setSpeed(intake, config.control.defaultIntakeSpeed)
      )
      .onFalse(commands.intake.setSpeed(intake, 0))
