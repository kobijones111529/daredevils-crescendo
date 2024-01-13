package frc.robot.scala.robot.subsystems.comp

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake extends SubsystemBase:
  private object intake:
    var speed = 0
    val primary: WPI_VictorSPX = ???
  
  override def periodic(): Unit =
    intake.primary.set(intake.speed)
