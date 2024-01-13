package frc.robot.subsystems.comp;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;

import java.util.Optional;

public class CompIntake extends SubsystemBase implements Intake {
  record PID<Err extends Unit<Err>, Out extends Unit<Out>>(
    Measure<Per<Out, Err>> p,
    Measure<Per<Out, Mult<Err, Time>>> i,
    Measure<Per<Out, Per<Err, Time>>> d
  ) {}

  static class PIDCon<Err extends Unit<Err>, Out extends Unit<Out>> {
    private record Measurement<Err extends Unit<Err>>(
      Measure<Err> err,
      Measure<Time> time,
      Measure<Mult<Err, Time>> acc
    ) {}

    public PID<Err, Out> pid;

    private Optional<Measurement<Err>> lastMeasure;

    public PIDCon(PID<Err, Out> pid) {
      this.pid = pid;
    }

    public Measure<Out> update(Measure<Err> error, Measure<Time> time) {
      @SuppressWarnings("unchecked")
      var acc = (Measure<Mult<Err, Time>>) error.times(BaseUnits.Time.zero());
      var rate = lastMeasure.map(last -> {
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

      lastMeasure = Optional.of(new Measurement<>(
        error, time, acc
      ));

      return p.plus(i).plus(d);
    }
  }

  public record Config(
    Intake intake,
    Actuate actuate
  ) {
    public record Intake(
      int primaryID
    ) {}

    public record Actuate(
      int primaryID,
      Sensors sensors
    ) {
      public sealed interface Sensors {
        record HasLimitSwitches(LimitSwitch up, LimitSwitch down) implements Sensors {}

        record HasEncoder(
          Encoder encoder,
          Optional<LimitSwitch> upLimitSwitch,
          Optional<LimitSwitch> downLimitSwitch,
          Measure<Angle> downPosition
        ) implements Sensors {}
      }

      record Encoder(
        int channelA,
        int channelB,
        Measure<Distance> distancePerPulse
      ) {}

      record LimitSwitch(int channel) {}
    }
  }

  private record IntakeGroup(
    WPI_VictorSPX primary
  ) {}

  private record Actuator(

  ) {
    public sealed interface HasAuto {
      void set(Actuate.Position position);
    }

    public sealed interface Type {}

    record Basic() implements Type {}
    public static final class WithLimitSwitches implements Type, HasAuto {
      record Sensors(
        DigitalInput up,
        DigitalInput down
      ) {}

      private final Sensors sensors;
      private Actuate.Position position;

      public WithLimitSwitches(Sensors sensors, Actuate.Position position) {
        this.sensors = sensors;
        this.position = position;
      }

      @Override
      public void set(Actuate.Position position) {
        this.position = position;
      }
    }
    record WithEncoder() implements Type {}

    record Motors(WPI_VictorSPX primary) {}
  }

  private record ActuateGroup(
    WPI_VictorSPX primary,
    Sensors sensors
  ) {
    public sealed interface HasControl<T> {
      void set(T control);
    }

    public sealed interface Sensors {
      record None() implements Sensors, HasControl<None.Control> {
        record Control(double output) {}
      }

      record HasLimitSwitches(DigitalInput up, DigitalInput down) implements Sensors {}

      record HasEncoder(
        Encoder encoder,
        Optional<DigitalInput> upLimitSwitch,
        Optional<DigitalInput> downLimitSwitch,
        Measure<Angle> downPosition,
        PIDCon<Angle, Dimensionless> pidController
      ) implements Sensors {}
    }
  }

  private final IntakeGroup intake;
  private final ActuateGroup actuate;

  private Actuate.Position position = Actuate.Position.Up;

  public CompIntake(Config config) {
    try (
      var intakePrimary = new WPI_VictorSPX(config.intake.primaryID);
      var actuatePrimary = new WPI_VictorSPX(config.actuate.primaryID);
    ) {
      ActuateGroup.Sensors sensors = switch (config.actuate.sensors) {
        case Config.Actuate.Sensors.HasLimitSwitches hasLimitSwitches ->
          new ActuateGroup.Sensors.HasLimitSwitches(
            new DigitalInput(hasLimitSwitches.up.channel),
            new DigitalInput(hasLimitSwitches.down.channel)
          );
        case Config.Actuate.Sensors.HasEncoder hasEncoder ->
          new ActuateGroup.Sensors.HasEncoder(
            new Encoder(hasEncoder.encoder.channelA, hasEncoder.encoder.channelB),
            hasEncoder.upLimitSwitch.map((limitSwitch) -> new DigitalInput(limitSwitch.channel)),
            hasEncoder.downLimitSwitch.map((limitSwitch -> new DigitalInput(limitSwitch.channel))),
            hasEncoder.downPosition,
            new PIDCon<>(
              new PID<>(
                Units.Value.of(1).per(Units.Degrees),
                Units.Value.of(0).per(Mult.combine(Units.Degrees, Units.Seconds)),
                Units.Value.of(0).per(Per.combine(Units.Degrees, Units.Seconds))
              )
            )
          );
      };

      intake = new IntakeGroup(intakePrimary);
      actuate = new ActuateGroup(actuatePrimary, sensors);
    }
  }

  @Override
  public void periodic() {
    switch (actuate.sensors) {
      case ActuateGroup.Sensors.None ignore -> {

      }
      case ActuateGroup.Sensors.HasLimitSwitches hasLimitSwitches -> {
        switch (position) {
          case Up -> {
            double speed = hasLimitSwitches.up.get() ? 0 : -1;
            actuate.primary.set(speed);
          }
          case Down -> {
            double speed = hasLimitSwitches.down.get() ? 0 : 1;
            actuate.primary.set(speed);
          }
        }
      }
      case ActuateGroup.Sensors.HasEncoder hasEncoder -> {
        // Reset encoder if top limit switch is engaged
        if (hasEncoder.upLimitSwitch.map(DigitalInput::get).orElse(false)) {
          hasEncoder.encoder.reset();
        }

        var target = switch (position) {
          case Up -> Units.Degrees.zero();
          case Down -> hasEncoder.downPosition;
        };
        var current = BaseUnits.Angle.of(hasEncoder.encoder.get());
        var error = current.minus(target);
        var time = Units.Seconds.of(Timer.getFPGATimestamp());
        var output = hasEncoder.pidController.update(error, time).in(Units.Value);

        actuate.primary.set(output);
      }
    }
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
