package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.statemachine.StateMachine;
import java.util.function.Supplier;

public class Arm extends StateMachine<Arm.State> {

  TalonFX armMotor = new TalonFX(50);
  TalonFX wristMotor = new TalonFX(51);

  CANcoder armEncoder = new CANcoder(52);
  CANcoder wristEncoder = new CANcoder(53);
  double armkFF;
  double wristkFF;

  CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
  CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();

  TalonFXConfiguration armConfig = new TalonFXConfiguration();
  TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  private final StatusSignal<Measure<Angle>> armAbsolutePosition = armEncoder.getAbsolutePosition();
  private final StatusSignal<Measure<Velocity<Angle>>> armSpeed = armEncoder.getVelocity();
  private final StatusSignal<Measure<Current>> armSupplyCurrent = armMotor.getSupplyCurrent();
  private final StatusSignal<Measure<Temperature>> armTemp = armMotor.getDeviceTemp();

  private final StatusSignal<Measure<Angle>> wristAbsolutePosition =
      wristEncoder.getAbsolutePosition();
  private final StatusSignal<Measure<Velocity<Angle>>> wristSpeed = wristEncoder.getVelocity();
  private final StatusSignal<Measure<Current>> wristSupplyCurrent = wristMotor.getSupplyCurrent();
  private final StatusSignal<Measure<Temperature>> wristTemp = wristMotor.getDeviceTemp();

  private static final Measure<Angle> AMP_WRIST = Degree.of(3.6);
  private static final Measure<Angle> TRAP_WRIST = Degree.of(3.6);
  private static final Measure<Angle> INTAKE_WRIST = Degree.of(3.6);
  private static final Measure<Angle> SOURCE_INTAKE_WRIST = Degree.of(3.6);
  private static final Measure<Angle> AMP_WRIST_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> TRAP_WRIST_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> INTAKE_WRIST_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> SPEAKER_WRIST_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> SOURCE_INTAKE_WRIST_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> AMP_ARM = Degree.of(3.6);
  private static final Measure<Angle> TRAP_ARM = Degree.of(3.6);
  private static final Measure<Angle> INTAKE_ARM = Degree.of(3.6);
  private static final Measure<Angle> SPEAKER_ARM = Degree.of(3.6);
  private static final Measure<Angle> SOURCE_INTAKE_ARM = Degree.of(1.0);
  private static final Measure<Angle> AMP_ARM_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> TRAP_ARM_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> INTAKE_ARM_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> SPEAKER_ARM_TOLERANCE = Degree.of(1.0);
  private static final Measure<Angle> SOURCE_INTAKE_ARM_TOLERANCE = Degree.of(1.0);

  private Supplier<Measure<Distance>> speakerDistance;

  public Arm(Supplier<Measure<Distance>> speakerDistance) {
    super("Arm", State.UNDETERMINED, State.class);
    this.speakerDistance = speakerDistance;
    registerStateCommands();
    registerTransitions();
    configureMotors();
  }

  private void configureMotors() {
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = -0.180908;

    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = 0.050244;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = armEncoder.getConfigurator().apply(armEncoderConfig, 0.1) == StatusCode.OK;
      statusOK =
          statusOK
              && wristEncoder.getConfigurator().apply(wristEncoderConfig, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    armConfig.Feedback.SensorToMechanismRatio = 1;

    armConfig.Slot0.kP = 160;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.Slot0.kG = 0.3;

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    wristConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristConfig.Feedback.SensorToMechanismRatio = 1;

    // -.2 //.2
    wristConfig.Slot0.kP = 68;
    wristConfig.Slot0.kI = 0;
    wristConfig.Slot0.kD = 0;
    wristConfig.Slot0.kG = 0;
    wristConfig.Slot0.kS = 0.216;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = wristMotor.getConfigurator().apply(wristConfig, 0.1) == StatusCode.OK;
      statusOK = statusOK && armMotor.getConfigurator().apply(armConfig, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armAbsolutePosition,
        armSpeed,
        armSupplyCurrent,
        armTemp,
        wristAbsolutePosition,
        wristSpeed,
        wristSupplyCurrent,
        wristTemp);
  }

  private void registerStateCommands() {
    registerStateCommand(State.IDLE, this::stop);
    registerStateCommand(
        State.INTAKE,
        new ParallelCommandGroup(
            new InstantCommand(() -> setArmAndWristTargets(INTAKE_ARM, INTAKE_WRIST)),
            atPositionCommand(
                () -> INTAKE_ARM,
                () -> INTAKE_WRIST,
                INTAKE_ARM_TOLERANCE,
                INTAKE_WRIST_TOLERANCE)));
    registerStateCommand(
        State.SPEAKER,
        new ParallelCommandGroup(
            new InstantCommand(() -> setArmAndWristTargets(SPEAKER_ARM, Degrees.of(0))),
            atPositionCommand(
                () -> SPEAKER_ARM,
                () -> Degrees.of(0),
                SPEAKER_ARM_TOLERANCE,
                SPEAKER_WRIST_TOLERANCE)));
    registerStateCommand(
        State.AMP,
        new ParallelCommandGroup(
            new InstantCommand(() -> setArmAndWristTargets(AMP_ARM, AMP_WRIST)),
            atPositionCommand(
                () -> AMP_ARM, () -> AMP_WRIST, AMP_ARM_TOLERANCE, AMP_WRIST_TOLERANCE)));
    registerStateCommand(
        State.TRAP,
        new ParallelCommandGroup(
            new InstantCommand(() -> setArmAndWristTargets(TRAP_ARM, TRAP_WRIST)),
            atPositionCommand(
                () -> TRAP_ARM, () -> TRAP_WRIST, TRAP_ARM_TOLERANCE, TRAP_WRIST_TOLERANCE)));
    registerStateCommand(
        State.SOURCE_INTAKE,
        new ParallelCommandGroup(
            new InstantCommand(() -> setArmAndWristTargets(SOURCE_INTAKE_ARM, SOURCE_INTAKE_WRIST)),
            atPositionCommand(
                () -> SOURCE_INTAKE_ARM,
                () -> SOURCE_INTAKE_WRIST,
                SOURCE_INTAKE_ARM_TOLERANCE,
                SOURCE_INTAKE_WRIST_TOLERANCE)));
  }

  private void registerTransitions() {
    addOmniTransition(State.SPEAKER);
    addOmniTransition(State.AMP);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.INTAKE);
    addOmniTransition(State.IDLE);
    addOmniTransition(State.SOURCE_INTAKE);
  }

  public void setArmAndWristTargets(Measure<Angle> armTarget, Measure<Angle> wristTarget) {
    setArmTarget(armTarget);
    setWristTarget(wristTarget);
  }

  public void setArmTarget(Measure<Angle> target) {
    var control = new PositionVoltage(0);
    armMotor.setControl(control.withPosition(target.in(Rotations)).withSlot(0).withEnableFOC(true));
  }

  public void setWristTarget(Measure<Angle> target) {
    var control = new PositionVoltage(0);
    wristMotor.setControl(
        control
            .withPosition(target.in(Rotations))
            .withSlot(0)
            .withEnableFOC(true)
            .withFeedForward(
                Rotation2d.fromRadians(armAbsolutePosition.getValue().in(Radians)).getCos()
                    * 0.173));
  }

  private Command atPositionCommand(
      Supplier<Measure<Angle>> armProvider,
      Supplier<Measure<Angle>> wristProvider,
      Measure<Angle> armAccuracy,
      Measure<Angle> wristAccuracy) {
    return new RunCommand(
        () -> {
          if (armProvider.get().minus(armAbsolutePosition.getValue()).magnitude()
              < armAccuracy.magnitude()) {
            setFlag(State.ARM_AT_POSITION);
          } else {
            clearFlag(State.ARM_AT_POSITION);
          }
          if (wristProvider.get().minus(wristAbsolutePosition.getValue()).magnitude()
              < wristAccuracy.magnitude()) {
            setFlag(State.WRIST_AT_POSITION);
          } else {
            clearFlag(State.WRIST_AT_POSITION);
          }
        });
  }

  private void stop() {
    armMotor.stopMotor();
    wristMotor.stopMotor();
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    INTAKE,
    SPEAKER,
    AMP,
    TRAP,
    SOURCE_INTAKE,

    // flags
    ARM_AT_POSITION,
    WRIST_AT_POSITION
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }
}
