package frc.robot.scala.robot.subsystems.comp

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake(val config: Intake.Config) extends SubsystemBase:
  private object intake:
    var speed = 0
    val primary: WPI_VictorSPX =
      WPI_VictorSPX(config.intake.primary)

  override def periodic(): Unit =
    intake.primary.set(intake.speed)

object Intake:
  case class Config(intake: Config.Intake)
  
  object Config:
    case class Intake(primary: Int)
  
  implicit val runIntake: Intake.Run[Intake] = new Intake.Run[Intake]:
    extension (intake: Intake)
      override def run(speed: Double): Unit =
        intake.intake.speed = speed
