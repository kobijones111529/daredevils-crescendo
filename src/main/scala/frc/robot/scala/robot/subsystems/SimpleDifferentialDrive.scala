package frc.robot.scala.robot.subsystems

trait SimpleDifferentialDrive[A]:
  extension (a: A) def stop(): Unit
  extension (a: A) def tankDrive(left: Double, right: Double): Unit
  extension (a: A) def arcadeDrive(move: Double, turn: Double): Unit
