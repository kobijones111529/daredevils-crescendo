package frc.robot.scala.robot.subsystems.comp

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.scala.robot.subsystems.SimpleDifferentialDrive
import frc.robot.scala.robot.subsystems.comp.Drivetrain.*

class Drivetrain(config: Config) extends SubsystemBase:
  private class DriveGroup(
      val primary: WPI_VictorSPX
  )

  private object drive:
    val left: DriveGroup = create(config.left)
    val right: DriveGroup = create(config.right)

    private def create(config: Config.DriveGroup): DriveGroup =
      val primary = WPI_VictorSPX(config.primaryID)
      val backups = config.backupIDs.map(id => WPI_VictorSPX(id))

      backups.foreach(backup => backup.follow(primary))

      DriveGroup(primary)
      
  end drive

  private var control: Control = Control.Stop

  override def periodic(): Unit =
    control match
      case Control.Stop =>
        drive.left.primary.set(0)
        drive.right.primary.set(0)
      case Control.TankDrive(left, right) =>
        val wheelSpeeds = DifferentialDrive.tankDriveIK(left, right, false)
        drive.left.primary.set(wheelSpeeds.left)
        drive.right.primary.set(wheelSpeeds.right)
      case Control.ArcadeDrive(move, turn) =>
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(move, turn, false)
        drive.left.primary.set(wheelSpeeds.left)
        drive.right.primary.set(wheelSpeeds.right)
    end match
  end periodic

object Drivetrain:
  case class Config(
      left: Config.DriveGroup,
      right: Config.DriveGroup
  )

  object Config:
    case class DriveGroup(
        primaryID: Int,
        backupIDs: Array[Int]
    )

  private enum Control:
    case Stop
    case TankDrive(left: Double, right: Double)
    case ArcadeDrive(move: Double, turn: Double)

  import Control.*

  implicit val simpleDifferentialDrive: SimpleDifferentialDrive[Drivetrain] =
    new SimpleDifferentialDrive[Drivetrain]:
      extension (drivetrain: Drivetrain)
        override def stop(): Unit =
          drivetrain.control = Stop
      extension (drivetrain: Drivetrain)
        override def tankDrive(left: Double, right: Double): Unit =
          drivetrain.control = TankDrive(left, right)
      extension (drivetrain: Drivetrain)
        override def arcadeDrive(move: Double, turn: Double): Unit =
          drivetrain.control = ArcadeDrive(move, turn)
