package frc.robot.scala.robot.subsystems

import scala.annotation.unused

trait SimpleDifferentialDrive[Drivetrain]:
  extension (@unused drivetrain: Drivetrain) def stop(): Unit
  extension (@unused drivetrain: Drivetrain) def tankDrive(left: Double, right: Double): Unit
  extension (@unused drivetrain: Drivetrain) def arcadeDrive(move: Double, turn: Double): Unit
