package frc.robot.subsystems.comp;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleIntake1 extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(10);
  private final WPI_VictorSPX actuateMotor = new WPI_VictorSPX(11);

  private final DigitalInput topLimitSwitch = new DigitalInput(4);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(5);

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setActuateSpeed(double speed) {
    if (topLimitSwitch.get()) {
      speed = Math.max(0, speed);
    }

    if (bottomLimitSwitch.get()) {
      speed = Math.min(speed, 0);
    }

    actuateMotor.set(speed);
  }
}
