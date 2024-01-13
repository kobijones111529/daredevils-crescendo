package frc.robot.scala.robot.commands

import edu.wpi.first.wpilibj2.command.{Command, Subsystem}
import frc.robot.scala.robot.subsystems.SimpleDifferentialDrive

import scala.annotation.unused

package object drive:
  def stop[Drivetrain <: Subsystem](drivetrain: Drivetrain)(implicit
      @unused impl: SimpleDifferentialDrive[Drivetrain]
  ): Command =
    drivetrain.runOnce(() => drivetrain.stop())

  def tank[Drivetrain <: Subsystem](
      drivetrain: Drivetrain,
      left: => Double,
      right: => Double
  )(implicit @unused impl: SimpleDifferentialDrive[Drivetrain]): Command =
    drivetrain
      .run(() => drivetrain.tankDrive(left, right))
      .finallyDo(() => drivetrain.stop())

  def arcade[Drivetrain <: Subsystem](
      drivetrain: Drivetrain,
      move: => Double,
      turn: => Double
  )(implicit @unused impl: SimpleDifferentialDrive[Drivetrain]): Command =
    drivetrain
      .run(() => drivetrain.arcadeDrive(move, turn))
      .finallyDo(() => drivetrain.stop())
  
