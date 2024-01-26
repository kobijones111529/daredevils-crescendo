package frc.robot.scala.robot.commands

import edu.wpi.first.wpilibj2.command.{Command, Subsystem}
import frc.robot.scala.robot.subsystems.intake.Run

import scala.annotation.unused

package object intake:
  def run[Intake <: Subsystem](intake: Intake, speed: => Double)(implicit
      @unused impl: Run[Intake]
  ): Command =
    intake.run(() => intake.run(speed)).finallyDo(() => intake.run(0))

  def setSpeed[Intake <: Subsystem](intake: Intake, speed: Double)(implicit
      @unused impl: Run[Intake]
  ): Command =
    intake.runOnce(() => intake.run(speed))
