package frc.robot.scala.robot

import frc.robot.scala.robot.subsystems.comp.Drivetrain

package object config:
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
