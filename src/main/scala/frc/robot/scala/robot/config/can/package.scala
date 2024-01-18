package frc.robot.scala.robot.config

package object can:
  object drivetrain:
    object left:
      val primary: Int = 0
      val backups: Array[Int] = Array(1)
    object right:
      val primary: Int = 2
      val backups: Array[Int] = Array(3)
  end drivetrain
  
  object intake:
    object intake:
      val primary: Int = 10
    object actuator:
      val primary: Int = 11
