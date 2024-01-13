package frc.robot.scala.robot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.scala.robot.subsystems.comp.Drivetrain

class Container:
  val auto: Option[Command] = None
  
  private val drivetrain: Drivetrain = Drivetrain(config.drivetrain)
  
  configureBindings()
  
  private def configureBindings(): Unit =
    drivetrain.setDefaultCommand(drivetrain.run(() => drivetrain.arcadeDrive(???, ???)))
  
end Container
