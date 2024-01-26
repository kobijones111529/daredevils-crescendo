package frc.robot.scala.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Instance extends TimedRobot:
  object Robot:
    private var _instance: Option[Container] = None

    def instance: Option[Container] = _instance
    def instance_=(value: Container): Unit =
      _instance = Some(value)

  end Robot

  override def robotInit(): Unit =
    Robot.instance = Container()

  override def robotPeriodic(): Unit =
    CommandScheduler.getInstance().run()

  override def autonomousInit(): Unit =
    for
      robot <- Robot.instance
      auto <- robot.auto
    yield
      CommandScheduler.getInstance().schedule(auto)
    end for

  override def teleopInit(): Unit =
    for
      robot <- Robot.instance
      auto <- robot.auto
    yield
      auto.cancel()
    end for

  override def testInit(): Unit =
    CommandScheduler.getInstance().cancelAll()

end Instance
