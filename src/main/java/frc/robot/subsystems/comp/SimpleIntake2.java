package frc.robot.subsystems.comp;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class SimpleIntake2 extends SubsystemBase {
  public enum Position {
    Up, Down
  }

  private sealed interface Control {
    record Manual(double speed) implements Control {}
    record Auto(Position position, double maxSpeed) implements Control {}
  }

  private final NetworkTableEntry intakeSpeedEntry;
  private final NetworkTableEntry actuatorManualSpeedEntry;
  private final NetworkTableEntry actuatorAutoPositionEntry;
  private final NetworkTableEntry actuatorActualSpeedEntry;

  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(10);
  private final WPI_VictorSPX actuateMotor = new WPI_VictorSPX(11);

  private final DigitalInput topLimitSwitch = new DigitalInput(4);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(5);

  private double intakeSpeed = 0;
  private Control actuateControl = new Control.Manual(0);

  public SimpleIntake2() {
    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Intake");
    intakeSpeedEntry = networkTable.getEntry("Intake speed (desired)");
    actuatorManualSpeedEntry = networkTable.getEntry("Actuator speed (manual)");
    actuatorAutoPositionEntry = networkTable.getEntry("Actuator position (auto)");
    actuatorActualSpeedEntry = networkTable.getEntry("Actuator speed (actual)");
  }

  @Override
  public void periodic() {
    intakeMotor.set(intakeSpeed);
    intakeSpeedEntry.setDouble(intakeSpeed);

    double actuateSpeed = actuateSpeed();
    actuateMotor.set(actuateSpeed);
    actuatorActualSpeedEntry.setDouble(actuateSpeed);
    switch (actuateControl) {
      case Control.Manual manual -> actuatorManualSpeedEntry.setDouble(manual.speed);
      case Control.Auto auto -> actuatorAutoPositionEntry.setString(switch (auto.position) {
        case Up -> "Up";
        case Down -> "Down";
      });
    }
  }

  private double actuateSpeed() {
    return switch (actuateControl) {
      case Control.Manual manual -> actuateSpeedManual(manual);
      case Control.Auto auto -> actuateSpeedAuto(auto);
    };
  }

  private double actuateSpeedManual(Control.Manual control) {
    double speed = control.speed;

    if (topLimitSwitch.get())
      speed = Math.max(0, speed);
    if (bottomLimitSwitch.get())
      speed = Math.min(speed, 0);

    return speed;
  }

  private double actuateSpeedAuto(Control.Auto auto) {
    return switch (auto.position) {
      case Down -> bottomLimitSwitch.get() ? 0 : Math.abs(auto.maxSpeed);
      case Up -> topLimitSwitch.get() ? 0 : -Math.abs(auto.maxSpeed);
    };
  }

  public void setIntakeSpeed(double speed) {
    intakeSpeed = speed;
  }

  public void setActuateSpeed(double speed) {
    actuateControl = new Control.Manual(speed);
  }

  public void setActuatePosition(Position position, double maxSpeed) {
    actuateControl = new Control.Auto(position, maxSpeed);
  }

  public Optional<Position> getPosition() {
    return switch (actuateControl) {
      case Control.Manual ignore -> Optional.empty();
      case Control.Auto auto -> Optional.of(auto.position);
    };
  }

  public boolean atTop() {
    return topLimitSwitch.get();
  }

  public boolean atBottom() {
    return bottomLimitSwitch.get();
  }
}
