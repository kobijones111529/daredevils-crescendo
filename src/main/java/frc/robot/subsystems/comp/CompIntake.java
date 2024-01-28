package frc.robot.subsystems.comp;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;

import java.util.Optional;

public class CompIntake extends SubsystemBase implements Intake {
  private Actuate.Position position = Actuate.Position.Up;

  public CompIntake() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void run(double speed) {
//    intake.primary.set(speed);
  }

  @Override
  public void setPosition(Actuate.Position position) {
    this.position = position;
  }

  @Override
  public Actuate.Position getPosition() {
    return position;
  }

  @Override
  public Actuate.Status getStatus() {
    return null;
  }
}
