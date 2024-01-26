package frc.robot.scala.robot.subsystems

import scala.annotation.unused

package object intake:
  object Actuator:
    enum Position:
      case Up, Down
  
  trait Run[Intake]:
    extension (@unused intake: Intake) def run(speed: Double): Unit
  
  trait Basic[Intake]:
    extension (@unused intake: Intake) def actuate(speed: Double): Unit

  trait Auto[Intake]:
    extension (@unused intake: Intake) def actuateTo(position: Actuator.Position): Unit
