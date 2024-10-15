package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.units.*;
import frc.robot.utils.statemachine.StateMachine;

public class Intake extends StateMachine<Intake.State> {
  // Actuation Motor
  CANSparkMax actuationMotor = new CANSparkMax(19, MotorType.kBrushless);
  SparkLimitSwitch upLimitSwitch =
      actuationMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  SparkLimitSwitch downLimitSwitch =
      actuationMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  // Roller Motor
  private final TalonFX rollerMotor = new TalonFX(30);
  private final StatusSignal<Measure<Angle>> rollerPosition = rollerMotor.getPosition();
  private final StatusSignal<Measure<Velocity<Angle>>> rollerVelocity = rollerMotor.getVelocity();
  private final StatusSignal<Measure<Voltage>> rollerAppliedVolts = rollerMotor.getMotorVoltage();
  private final StatusSignal<Measure<Current>> rollerCurrent = rollerMotor.getSupplyCurrent();
  TalonFXConfiguration config = new TalonFXConfiguration();

  public Intake() {
    super("Intake", State.UNDETERMINED, State.class);
    registerStateCommands();
    registerTransitions();

    configureMotors();
  }

  private void registerStateCommands() {
    registerStateCommand(State.UNDETERMINED, () -> {});
    registerStateCommand(State.SOFT_E_STOP, () -> setActuatorVoltage(0));
    registerStateCommand(
        State.INTAKE_UP,
        () -> {
          intakeUp();
          intakeStop();
        });
    registerStateCommand(
        State.INTAKE_DOWN,
        () -> {
          intakeDown();
          intakeFast();
        });
    registerStateCommand(
        State.OUTTAKE_UP,
        () -> {
          intakeUp();
          intakeReverse();
        });
    registerStateCommand(
        State.OUTTAKE_DOWN,
        () -> {
          intakeDown();
          intakeReverse();
        });
  }

  private void registerTransitions() {
    addTransition(State.UNDETERMINED, State.SOFT_E_STOP);
    addTransition(State.SOFT_E_STOP, State.INTAKE_UP);
    addTransition(State.SOFT_E_STOP, State.INTAKE_DOWN);
    addTransition(State.INTAKE_UP, State.INTAKE_DOWN);
    addTransition(State.INTAKE_DOWN, State.INTAKE_UP);
    addCommutativeTransition(State.INTAKE_DOWN, State.OUTTAKE_DOWN);
    addCommutativeTransition(State.INTAKE_DOWN, State.OUTTAKE_UP);
    addCommutativeTransition(State.INTAKE_UP, State.OUTTAKE_UP);
    addCommutativeTransition(State.INTAKE_UP, State.OUTTAKE_DOWN);
  }

  private void intakeUp() {
    if (upLimitSwitch.isPressed()) {
      setActuatorVoltage(-0.01);
    } else {
      setActuatorVoltage(-4.0);
    }
    actuationMotor.setIdleMode(IdleMode.kBrake);
  }

  private void intakeFast() {
    setSpeed(RotationsPerSecond.of(80));
  }

  private void intakeSlow() {
    setSpeed(RotationsPerSecond.of(40));
  }

  private void intakeReverse() {
    setSpeed(RotationsPerSecond.of(-80));
  }

  private void intakeStop() {
    setSpeed(RotationsPerSecond.of(0));
  }

  private void setSpeed(Measure<Velocity<Angle>> velocity) {
    new VelocityTorqueCurrentFOC(velocity.in(RotationsPerSecond));
  }

  private void intakeDown() {
    if (downLimitSwitch.isPressed()) {
      setActuatorVoltage(0.01);
    } else {
      setActuatorVoltage(4.0);
    }
    actuationMotor.setIdleMode(IdleMode.kCoast);
  }

  private void setActuatorVoltage(double voltage) {
    actuationMotor.setVoltage(voltage);
  }

  private void configureMotors() {
    actuationMotor.setInverted(false);
    actuationMotor.setIdleMode(IdleMode.kCoast);
    actuationMotor.setSmartCurrentLimit(20);

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = 13;
    config.Slot0.kI = 5;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0.21;
    config.Slot0.kS = 24;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = rollerMotor.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    INTAKE_UP,
    INTAKE_DOWN,
    OUTTAKE_UP,
    OUTTAKE_DOWN,
  }
}
