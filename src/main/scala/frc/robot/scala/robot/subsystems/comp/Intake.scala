package frc.robot.scala.robot.subsystems.comp

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj.{DigitalInput, Encoder as WPI_Encoder}
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.scala.robot.subsystems.comp.Intake.Config
import frc.robot.scala.robot.subsystems.intake.Actuator.Position
import frc.robot.scala.robot.subsystems.intake.Run

import scala.math.*

class Intake(val config: Intake.Config) extends SubsystemBase:
  private case class Actuator(
      primary: WPI_VictorSPX,
      sensors: Actuator.WithSensors
  )
  private object Actuator:
    sealed trait WithSensors
    object WithSensors:
      case class Basic(
          limitSwitch: Option[(Position, DigitalInput)],
          var speed: Double = 0
      ) extends WithSensors
      case class LimitSwitches(
          up: DigitalInput,
          down: DigitalInput,
          control: LimitSwitches.Control
      ) extends WithSensors
      object LimitSwitches:
        enum Control:
          case Manual(speed: Double)
          case Auto(position: Position)
      case class Encoder(
          encoder: WPI_Encoder,
          up: Option[DigitalInput],
          down: Option[DigitalInput],
          control: Encoder.Control
      ) extends WithSensors
      object Encoder:
        enum Control:
          case Manual(speed: Double)
          case Auto(position: Position)
  end Actuator

  private object intake:
    var speed: Double = 0
    val primary: WPI_VictorSPX =
      WPI_VictorSPX(config.intake.primary)

  private val actuator: Actuator =
    val primary = WPI_VictorSPX(config.actuator.primaryID)
    val sensors: Actuator.WithSensors = config.actuator.sensors match
      case Config.Actuator.Sensors.Basic(limitSwitch) =>
        Actuator.WithSensors.Basic(
          limitSwitch.map((position, channel) =>
            (position, DigitalInput(channel))
          )
        )
      case Config.Actuator.Sensors.LimitSwitches(upChannel, downChannel) =>
        Actuator.WithSensors.LimitSwitches(
          up = DigitalInput(upChannel),
          down = DigitalInput(downChannel),
          control = ???
        )
      case Config.Actuator.Sensors.Encoder(
            encoderChannelA,
            encoderChannelB,
            upLimitSwitch,
            downLimitSwitch
          ) =>
        Actuator.WithSensors.Encoder(
          encoder = WPI_Encoder(encoderChannelA, encoderChannelB),
          up = upLimitSwitch.map(channel => DigitalInput(channel)),
          down = downLimitSwitch.map(channel => DigitalInput(channel)),
          control = ???
        )
    Actuator(primary, sensors)
  end actuator

  override def periodic(): Unit =
    intake.primary.set(intake.speed)

    val actuateSpeed: Double = actuator.sensors match
      case basic: Actuator.WithSensors.Basic =>
        getActuateSpeed(basic.limitSwitch, basic.speed)
      case limitSwitches: Actuator.WithSensors.LimitSwitches =>
        import Actuator.WithSensors.LimitSwitches.Control.*
        limitSwitches.control match
          case control: Manual =>
            var speed: Double = control.speed
            if limitSwitches.up.get() then speed = max(0, speed)
            if limitSwitches.down.get() then speed = min(speed, 0)
            speed
          case control: Auto => ???
      case _: Actuator.WithSensors.Encoder => ???
    end actuateSpeed

    actuator.primary.set(actuateSpeed)
  end periodic

  private def getActuateSpeed(
      limitSwitch: Option[(Position, DigitalInput)],
      desiredSpeed: Double
  ): Double =
    limitSwitch match
      case None => desiredSpeed
      case Some(Position.Up, limitSwitch) =>
        if limitSwitch.get() then max(0, desiredSpeed)
        else desiredSpeed
      case Some(Position.Down, limitSwitch) =>
        if limitSwitch.get() then min(desiredSpeed, 0)
        else desiredSpeed

object Intake:
  case class Config(intake: Config.Intake, actuator: Config.Actuator)

  object Config:
    case class Intake(primary: Int)
    case class Actuator(primaryID: Int, sensors: Actuator.Sensors)
    object Actuator:
      enum Sensors:
        case Basic(limitSwitch: Option[(Position, Int)])
        case LimitSwitches(up: Int, down: Int)
        case Encoder(
            encoderChannelA: Int,
            encoderChannelB: Int,
            upLimitSwitch: Option[Int],
            downLimitSwitch: Option[Int]
        )

  implicit val runIntake: Run[Intake] = new Run[Intake]:
    extension (intake: Intake)
      override def run(speed: Double): Unit =
        intake.intake.speed = speed
