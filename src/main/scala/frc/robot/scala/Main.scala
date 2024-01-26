package frc.robot.scala

import edu.wpi.first.wpilibj.RobotBase
import frc.robot.scala.robot.Instance as Robot

object Main:
  def main(args: Array[String]): Unit =
    RobotBase.startRobot(() => Robot)
