package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.MotorIOInputsAutoLogged;
import frc.robot.subsystems.base.Setpoint;
import frc.robot.util.logging.Elastic;
import frc.robot.util.logging.Elastic.Notification;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Climb extends StateMachine<Climb.State> {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  private final BeamBreakerIO leftSensor;
  private final BeamBreakerIO rightSensor;
  private final BeamBreakerInputsAutoLogged leftInputs = new BeamBreakerInputsAutoLogged();
  private final BeamBreakerInputsAutoLogged rightInputs = new BeamBreakerInputsAutoLogged();

  public static final Setpoint STOW = Setpoint.motionMagicPosition(ClimbConstants.kClimbStowPos);
  public static final Setpoint UP = Setpoint.motionMagicPosition(ClimbConstants.kClimbUpPos);

  private static final int kActionSlot = 1;

  public Climb(MotorIO io, BeamBreakerIO leftSensor, BeamBreakerIO rightSensor) {
    super("Climb", State.UNDETERMINED, State.class);
    this.io = io;
    this.leftSensor = leftSensor;
    this.rightSensor = rightSensor;

    addOmniTransitions(State.STOW, State.IDLE, State.UP, State.DOWN);

    registerStateCommand(State.IDLE, () -> io.stop());
    registerStateCommand(State.STOW, () -> STOW.apply(io));
    registerStateCommand(State.UP, () -> UP.apply(io));
    registerStateCommand(
        State.DOWN, () -> io.setMotionMagicPosition(ClimbConstants.kClimbDownPos, kActionSlot));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    leftSensor.updateInputs(leftInputs);
    rightSensor.updateInputs(rightInputs);
    Logger.processInputs("Climb/LeftSensor", leftInputs);
    Logger.processInputs("Climb/RightSensor", rightInputs);
  }

  @Override
  protected void determineSelf() {
    setState(State.STOW);
  }

  public Command zero() {
    return run(() -> io.setDutyCycle(ClimbConstants.kLowerMotorOutput))
        .beforeStarting(() -> io.setCurrentLimit(ClimbConstants.kLowerCurrentLimit))
        .until(() -> inputs.currentAmps > ClimbConstants.kZeroCurrentThreshold)
        .finallyDo(
            interrupted -> {
              io.stop();
              io.setEncoderPosition(0);
              io.setCurrentLimit(ClimbConstants.kClimbCurrentLimit);
              requestTransition(State.STOW);
              Elastic.sendNotification(
                  new Notification()
                      .withTitle("Climb Zero")
                      .withDescription("Climb has been zeroed!"));
            })
        .withName("Climb Zero");
  }

  public double getLeftSensorDistance() {
    return leftSensor.getDistance();
  }

  public double getRightSensorDistance() {
    return rightSensor.getDistance();
  }

  public enum State {
    UNDETERMINED,
    STOW,
    IDLE,
    UP,
    DOWN
  }
}
