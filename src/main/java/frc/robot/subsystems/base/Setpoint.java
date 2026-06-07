package frc.robot.subsystems.base;

import java.util.function.Consumer;
import java.util.function.ObjDoubleConsumer;

public class Setpoint {
  private final String name;
  private final double value;
  private final Consumer<MotorIO> action;
  private final ObjDoubleConsumer<MotorIO> ffAction;

  private Setpoint(String name, double value, Consumer<MotorIO> action, ObjDoubleConsumer<MotorIO> ffAction) {
    this.name = name;
    this.value = value;
    this.action = action;
    this.ffAction = ffAction;
  }

  public static Setpoint motionMagicPosition(double positionRad) {
    return new Setpoint(
        "MotionMagic",
        positionRad,
        io -> io.setMotionMagicPosition(positionRad),
        (io, ff) -> io.setMotionMagicPosition(positionRad, ff));
  }

  public static Setpoint motionMagic(double positionRad) {
    return motionMagicPosition(positionRad);
  }

  public static Setpoint position(double positionRad) {
    return new Setpoint(
        "Position",
        positionRad,
        io -> io.setPosition(positionRad),
        (io, ff) -> io.setPosition(positionRad, ff));
  }

  public static Setpoint velocity(double velocityRadPerSec) {
    return new Setpoint(
        "Velocity",
        velocityRadPerSec,
        io -> io.setVelocity(velocityRadPerSec),
        (io, ff) -> io.setVelocity(velocityRadPerSec, ff));
  }

  public static Setpoint motionMagicVelocity(double velocityRadPerSec) {
    return new Setpoint(
        "MotionMagicVelocity",
        velocityRadPerSec,
        io -> io.setMotionMagicVelocity(velocityRadPerSec),
        (io, ff) -> io.setMotionMagicVelocity(velocityRadPerSec, ff));
  }

  public static Setpoint voltage(double volts) {
    return new Setpoint("Voltage", volts, io -> io.setVoltage(volts), (io, ff) -> io.setVoltage(volts));
  }

  public static Setpoint idle() {
    return new Setpoint("Idle", 0.0, MotorIO::stop, (io, ff) -> io.stop());
  }

  public void apply(MotorIO io) {
    action.accept(io);
  }

  public void applyWithFeedforward(MotorIO io, double feedforwardVolts) {
    ffAction.accept(io, feedforwardVolts);
  }

  public double getValue() {
    return value;
  }

  public String getName() {
    return name;
  }
}
