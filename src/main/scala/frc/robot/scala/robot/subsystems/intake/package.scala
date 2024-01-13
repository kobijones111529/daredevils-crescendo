package frc.robot.scala.robot.subsystems

package object intake:
  trait Basic[Intake]:
    extension (intake: Intake) def run(speed: Double): Unit
    extension (intake: Intake) def actuate(speed: Double): Unit
