package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utils.statemachine.StateMachine;

public class Climber extends StateMachine<Climber.State> {

  private final TalonFX climberMotor = new TalonFX(42, "chassis");
  private final DigitalInput limitSwitch = new DigitalInput(1);

  private final StatusSignal<Measure<Angle>> climberPosition = climberMotor.getPosition();
  private final StatusSignal<Measure<Velocity<Angle>>> climberVelocity = climberMotor.getVelocity();
  private final StatusSignal<Measure<Voltage>> climberAppliedVolts = climberMotor.getMotorVoltage();
  private final StatusSignal<Measure<Current>> climberCurrent = climberMotor.getSupplyCurrent();

  final DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, 2, 10, 100);

  TalonFXConfiguration config = new TalonFXConfiguration();

  public Climber() {
    super("Climber", State.UNDETERMINED, State.class);

    registerStateCommands();
    registerTransitions();

    configureMotors();
  }

  private void configureMotors() {
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 25;

    config.Slot0.kS = 0.25; // .25V to overcome static friction
    config.Slot0.kV = 1.0 / 6.7;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 8.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.Slot1.kS = 0.228;
    config.Slot1.kV = 1.0 / 6.7;
    config.Slot1.kA = 0.0;
    config.Slot1.kP = 30.0;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = 0.0;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = climberMotor.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.RESETTING,
        Commands.deadline(
            Commands.waitUntil(this::isLimitSwitchTriggered),
            Commands.runEnd(() -> voltageControl(Volts.of(-2)), this::stopMotor, this)));

    registerStateCommand(
        State.STOWED, Commands.runOnce(() -> moveToSetpoint(STOWED_POSITION), this));

    registerStateCommand(
        State.ARMS_UP, Commands.runOnce(() -> moveToSetpoint(ARMS_UP_POSITION), this));

    registerStateCommand(
        State.QUICK_CLIMB, Commands.runOnce(() -> moveToSetpoint(QUICK_CLIMB_POSITION), this));

    registerStateCommand(
        State.TRAP_CLIMB, Commands.runOnce(() -> moveToSetpoint(ARMS_UP_POSITION), this));
  }

  private void registerTransitions() {
    addOmniTransition(State.RESETTING);
    addTransition(State.RESETTING, State.STOWED);
    addCommutativeTransition(State.STOWED, State.ARMS_UP);
    addCommutativeTransition(State.ARMS_UP, State.QUICK_CLIMB);
    addCommutativeTransition(State.ARMS_UP, State.TRAP_CLIMB);
  }

  @Override
  protected void determineSelf() {
    setState(State.RESETTING);
  }

  @Override
  protected void update() {}

  public boolean isLimitSwitchTriggered() {
    return limitSwitch.get();
  }

  /**
   * Move to the target setpoint.
   *
   * <p>Defaults to the PID values for when the robot is not hanging
   *
   * @param setpoint
   */
  private void moveToSetpoint(Measure<Distance> setpoint) {
    moveToSetpoint(setpoint, 0);
  }

  /**
   * Move to the target setpoint.
   *
   * @param setpoint
   * @param slot
   */
  private void moveToSetpoint(Measure<Distance> setpoint, int slot) {
    // TODO: Logic for slowing down the climb as the robot catches under the stage
    climberMotor.setControl(
        request.withPosition(calculateAngleFromSetpoint(setpoint)).withSlot(slot));
  }

  private void voltageControl(Measure<Voltage> appliedVoltage) {
    climberMotor.setControl(new VoltageOut(appliedVoltage.in(Volts)));
  }

  private void stopMotor() {
    climberMotor.stopMotor();
  }

  private Measure<Angle> calculateAngleFromSetpoint(Measure<Distance> setpoint) {
    // Zero the position to the minimum position
    Measure<Distance> zeroedPosition = setpoint.minus(MINIMUM_POSITION);

    // Use the circumference of the wheel to calculate the angle
    Measure<Distance> circumfrence = SPOOL_DIAMETER.times(Math.PI);
    Measure<Angle> angle = Rotations.of(zeroedPosition.in(Inch) / circumfrence.in(Inch));

    return angle;
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    RESETTING,
    STOWED,
    ARMS_UP,
    QUICK_CLIMB,
    TRAP_CLIMB
  }
}
