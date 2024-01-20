package frc.robot.subsystems.comp;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;
import java.util.Optional;

public class CompIntake extends SubsystemBase implements Intake {
  record PID<Err extends Unit<Err>, Out extends Unit<Out>>(
      Measure<Per<Out, Err>> p,
      Measure<Per<Out, Mult<Err, Time>>> i,
      Measure<Per<Out, Per<Err, Time>>> d) {}

  static class PIDCon<Err extends Unit<Err>, Out extends Unit<Out>> {
    private record Measurement<Err extends Unit<Err>>(
        Measure<Err> err, Measure<Time> time, Measure<Mult<Err, Time>> acc) {}

    public PID<Err, Out> pid;

    private Optional<Measurement<Err>> lastMeasure;

    public PIDCon(PID<Err, Out> pid) {
      this.pid = pid;
    }

    public Measure<Out> update(Measure<Err> error, Measure<Time> time) {
      @SuppressWarnings("unchecked")
      var acc = (Measure<Mult<Err, Time>>) error.times(BaseUnits.Time.zero());
      var rate =
          lastMeasure
              .map(
                  last -> {
                    var errDiff = error.minus(last.err);
                    var timeDiff = time.minus(last.time);

                    // Divide by zero
                    if (timeDiff.isEquivalent(BaseUnits.Time.zero())) {
                      return errDiff.times(0).per(BaseUnits.Time);
                    }

                    return errDiff.per(timeDiff);
                  })
              .orElse(error.times(0).per(BaseUnits.Time));

      @SuppressWarnings("unchecked")
      var p = (Measure<Out>) pid.p.times(error);
      @SuppressWarnings("unchecked")
      var i = (Measure<Out>) pid.i.times(acc);
      @SuppressWarnings("unchecked")
      var d = (Measure<Out>) pid.d.times(rate);

      lastMeasure = Optional.of(new Measurement<>(error, time, acc));

      return p.plus(i).plus(d);
    }
  }

  public record Config(Intake intake, Actuator actuator) {
    public record Intake(int primaryID) {}

    public record Actuator(int primaryID, Type type) {
      public sealed interface Type {}

      record Basic(Sensors sensors) implements Type {
        public sealed interface Sensors {
          record None() implements Sensors {}

          record Top(LimitSwitch limitSwitch) implements Sensors {}

          record Bottom(LimitSwitch limitSwitch) implements Sensors {}
        }
      }

      record WithLimitSwitches(LimitSwitch up, LimitSwitch down) implements Type {}

      record WithEncoder(
          Encoder encoder,
          Optional<LimitSwitch> upLimitSwitch,
          Optional<LimitSwitch> downLimitSwitch,
          Measure<Angle> downPosition)
          implements Type {}

      record Encoder(int channelA, int channelB, Measure<Distance> distancePerPulse) {}

      record LimitSwitch(int channel) {}
    }
  }

  private record Intake(WPI_VictorSPX primary) {}

  private record Actuator(Motors motors, Type type) {
    public sealed interface Type {}

    static final class Basic implements Type {
      public sealed interface Sensors {
        record None() implements Sensors {}

        record Top(DigitalInput limitSwitch) implements Sensors {}

        record Bottom(DigitalInput limitSwitch) implements Sensors {}
      }

      public final Sensors sensors;
      public double speed;

      public Basic(Sensors sensors) {
        this(sensors, 0);
      }

      public Basic(Sensors sensors, double speed) {
        this.sensors = sensors;
        this.speed = speed;
      }
    }

    public static final class WithLimitSwitches implements Type {
      record Sensors(DigitalInput top, DigitalInput bottom) {}

      public sealed interface Control {
        record Manual(double speed) implements Control {}

        record Auto(Actuate.Position position) implements Control {}
      }

      public final Sensors sensors;
      public Control control;

      public WithLimitSwitches(Sensors sensors, Control control) {
        this.sensors = sensors;
        this.control = control;
      }
    }

    public static final class WithEncoder implements Type {
      record Sensors(Encoder encoder, Optional<DigitalInput> top, Optional<DigitalInput> bottom) {}

      public sealed interface Control {
        record Manual(double speed) implements Control {}

        record Auto(Actuate.Position position) implements Control {}
      }

      public final Sensors sensors;
      public Control control;

      public WithEncoder(Sensors sensors, Control control) {
        this.sensors = sensors;
        this.control = control;
      }
    }

    record Motors(WPI_VictorSPX primary) {}
  }

  private final Intake intake;
  private final Actuator actuator;

  private Actuate.Position position = Actuate.Position.Up;

  public CompIntake(Config config) {
    Actuator.Type type =
        switch (config.actuator.type) {
          case Config.Actuator.Basic basic -> createActuatorBasic(basic);
          case Config.Actuator.WithLimitSwitches withLimitSwitches ->
              createActuatorWithLimitSwitches(withLimitSwitches);
          case Config.Actuator.WithEncoder withEncoder -> createActuatorWithEncoder(withEncoder);
        };

    intake = new Intake(new WPI_VictorSPX(config.intake.primaryID));
    actuator =
        new Actuator(new Actuator.Motors(new WPI_VictorSPX(config.actuator.primaryID)), type);
  }

  private Actuator.Basic createActuatorBasic(Config.Actuator.Basic config) {
    var sensors =
        switch (config.sensors) {
          case Config.Actuator.Basic.Sensors.None ignore -> new Actuator.Basic.Sensors.None();
          case Config.Actuator.Basic.Sensors.Top top -> {
            var limitSwitch = new DigitalInput(top.limitSwitch.channel);
            yield new Actuator.Basic.Sensors.Top(limitSwitch);
          }
          case Config.Actuator.Basic.Sensors.Bottom bottom -> {
            var limitSwitch = new DigitalInput(bottom.limitSwitch.channel);
            yield new Actuator.Basic.Sensors.Bottom(limitSwitch);
          }
        };
    return new Actuator.Basic(sensors, 0);
  }

  private Actuator.WithLimitSwitches createActuatorWithLimitSwitches(
      Config.Actuator.WithLimitSwitches config) {
    var top = new DigitalInput(config.up.channel);
    var bottom = new DigitalInput(config.down.channel);
    var sensors = new Actuator.WithLimitSwitches.Sensors(top, bottom);
    return new Actuator.WithLimitSwitches(
        sensors, new Actuator.WithLimitSwitches.Control.Manual(0));
  }

  private Actuator.WithEncoder createActuatorWithEncoder(Config.Actuator.WithEncoder config) {
    var encoder = new Encoder(config.encoder.channelA, config.encoder.channelB);
    encoder.setDistancePerPulse(config.encoder.distancePerPulse.baseUnitMagnitude());

    var top = config.upLimitSwitch.map(limitSwitch -> new DigitalInput(limitSwitch.channel));
    var bottom = config.downLimitSwitch.map(limitSwitch -> new DigitalInput(limitSwitch.channel));
    var sensors = new Actuator.WithEncoder.Sensors(encoder, top, bottom);
    return new Actuator.WithEncoder(sensors, new Actuator.WithEncoder.Control.Manual(0));
  }

  @Override
  public void periodic() {
    switch (actuator.type) {
      case Actuator.Basic basic -> {
        double speed = actuateSpeed(basic);
        actuator.motors.primary.set(speed);
      }
      case Actuator.WithLimitSwitches withLimitSwitches -> {
        double speed = actuateSpeed(withLimitSwitches);
        actuator.motors.primary.set(speed);
      }
      case Actuator.WithEncoder withEncoder -> {
        // TODO
      }
    }
  }

  private double actuateSpeed(Actuator.Basic basic) {
    double desiredSpeed = basic.speed;

    return switch (basic.sensors) {
      case Actuator.Basic.Sensors.None ignored -> desiredSpeed;
      case Actuator.Basic.Sensors.Top top -> {
        if (top.limitSwitch.get()) yield Math.max(0, desiredSpeed);
        else yield desiredSpeed;
      }
      case Actuator.Basic.Sensors.Bottom bottom -> {
        if (bottom.limitSwitch.get()) yield Math.min(desiredSpeed, 0);
        else yield desiredSpeed;
      }
    };
  }

  private double actuateSpeed(Actuator.WithLimitSwitches withLimitSwitches) {
    var sensors = withLimitSwitches.sensors;

    return switch (withLimitSwitches.control) {
      case Actuator.WithLimitSwitches.Control.Manual manual -> {
        double speed = manual.speed;

        if (sensors.top.get()) speed = Math.max(0, speed);

        if (sensors.bottom.get()) speed = Math.min(speed, 0);

        yield speed;
      }
      case Actuator.WithLimitSwitches.Control.Auto auto ->
          switch (auto.position) {
            case Up -> sensors.top.get() ? 0 : -1;
            case Down -> sensors.bottom.get() ? 0 : 1;
          };
    };
  }

  @Override
  public void run(double speed) {
    intake.primary.set(speed);
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
