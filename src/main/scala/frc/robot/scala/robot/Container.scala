package frc.robot.scala.robot

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.scala.robot.subsystems.comp.Drivetrain

import scala.math.*

class Container:
  val auto: Option[Command] = None

  private val drivetrain: Drivetrain = Drivetrain(config.drivetrain)

  private val xbox: CommandXboxController = CommandXboxController(
    config.driverStation.xbox
  )

  configureBindings()

  private def configureBindings(): Unit =
    drivetrain.setDefaultCommand(
      commands.drive.arcade(drivetrain, xbox.getLeftY, xbox.getRightX)
    )
