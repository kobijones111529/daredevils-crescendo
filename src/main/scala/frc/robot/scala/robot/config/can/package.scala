package frc.robot.scala.robot.config

package object can:
  object drivetrain:
    object left:
      val primary = 0
      val backups: Array[Int] = Array(1)
    object right:
      val primary = 2
      val backups: Array[Int] = Array(3)
