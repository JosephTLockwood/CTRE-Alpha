package frc.robot.subsystems.magazine;

import com.revrobotics.CANSparkMax;
import frc.robot.utils.statemachine.StateMachine;

public class Magazine extends StateMachine<Magazine.State> {

  private final CANSparkMax magazineMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);

  public Magazine() {
    super("Magazine", State.UNDETERMINED, State.class);
    configureMotors();
    registerStateCommands();
    registerTransitions();
  }

  private void configureMotors() {
    magazineMotor.setInverted(false);
    magazineMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    magazineMotor.setSmartCurrentLimit(30);
  }

  private void registerStateCommands() {
    registerStateCommand(State.IDLE, this::stop);
    registerStateCommand(State.FEEDING, this::feed);
    registerStateCommand(State.REVERSING, this::reverse);
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.FEEDING);
    addOmniTransition(State.REVERSING);
  }

  private void setVoltage(double voltage) {
    magazineMotor.setVoltage(voltage);
  }

  public void feed() {
    setVoltage(6);
  }

  public void reverse() {
    setVoltage(-7);
  }

  public void stop() {
    setVoltage(0);
  }

  @Override
  protected void determineSelf() {
    setState(State.UNDETERMINED);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    FEEDING,
    REVERSING
  }
}
