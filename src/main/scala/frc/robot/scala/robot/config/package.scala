package frc.robot.scala.robot

import frc.robot.scala.robot.subsystems.comp.{Drivetrain, Intake}

package object config:
  object control:
    val defaultIntakeSpeed: Double = 0.5

  object driverStation:
    val xbox: Int = 0

  val drivetrain: Drivetrain.Config = Drivetrain.Config(
    left = Drivetrain.Config.DriveGroup(
      primaryID = can.drivetrain.left.primary,
      backupIDs = can.drivetrain.left.backups
    ),
    right = Drivetrain.Config.DriveGroup(
      primaryID = can.drivetrain.right.primary,
      backupIDs = can.drivetrain.right.backups
    )
  )

  val intake: Intake.Config = Intake.Config(
    Intake.Config.Intake(can.intake.intake.primary),
    Intake.Config.Actuator(
      primaryID = can.intake.actuator.primary,
      sensors =
        import Intake.Config.Actuator.Sensors.*
        Basic(limitSwitch = None)
    )
  )
