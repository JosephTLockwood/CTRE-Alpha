// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.statemachine.StateMachine;
import java.util.function.Supplier;

public class Flywheel extends StateMachine<Flywheel.State> {

  private final TalonFX leader = new TalonFX(53);
  private final TalonFX follower = new TalonFX(54);

  private final StatusSignal<Measure<Angle>> leaderPosition = leader.getPosition();
  private final StatusSignal<Measure<Velocity<Angle>>> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Measure<Voltage>> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Measure<Current>> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Measure<Current>> followerCurrent = follower.getSupplyCurrent();

  final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  TalonFXConfiguration config = new TalonFXConfiguration();

  private static final Measure<Velocity<Angle>> AMP_SHOT = RotationsPerSecond.of(20.0);
  private static final Measure<Velocity<Angle>> TRAP_SHOT = RotationsPerSecond.of(3.6);
  private static final Measure<Velocity<Angle>> SUBWOOFER_SHOT = RotationsPerSecond.of(3.6);

  private static final Measure<Velocity<Angle>> AMP_TOLERANCE = RotationsPerSecond.of(3.0);
  private static final Measure<Velocity<Angle>> TRAP_TOLERANCE = RotationsPerSecond.of(3.6);
  private static final Measure<Velocity<Angle>> SUBWOOFER_TOLERANCE = RotationsPerSecond.of(3.6);
  private static final Measure<Velocity<Angle>> SPEAKER_TOLERANCE = RotationsPerSecond.of(3.6);

  private Supplier<Measure<Velocity<Angle>>> speaker;

  public Flywheel(Supplier<Measure<Velocity<Angle>>> speaker) {
    super("Flywheel", State.UNDETERMINED, State.class);
    this.speaker = speaker;
    registerStateCommands();
    registerTransitions();
    configureMotors();
  }

  private void configureMotors() {
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // set slot 0 gains

    // kV assumes linear however get to operating velocity

    config.Slot0.kS = 0.366; // Add 0.31 V output to overcome static friction
    config.Slot0.kV = 8.0 / 54.154; // 7.25 / 50.0
    config.Slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    config.Slot0.kP = 0.9; // An error of 1 rps results in 0.11 V output
    config.Slot0.kI = 0.0; // no output for integrated error
    config.Slot0.kD = 0.001; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = config.MotionMagic;
    // Some value that is achievable
    motionMagicConfigs.MotionMagicAcceleration =
        50.0; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 500; // Target jerk of 4000 rps/s/s (0.1 seconds)

    for (int i = 0; i < 4; i++) {
      boolean statusOK = leader.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      statusOK = statusOK && follower.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }
    follower.setControl(new Follower(leader.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.AMP,
        new ParallelCommandGroup(
            new InstantCommand(() -> setFlywheelTarget(AMP_SHOT)),
            atSpeedCommand(() -> AMP_SHOT, AMP_TOLERANCE)));
    registerStateCommand(
        State.SPEAKER,
        new ParallelCommandGroup(
            new RunCommand(() -> setFlywheelTarget(speaker.get())),
            atSpeedCommand(speaker, SPEAKER_TOLERANCE)));

    registerStateCommand(
        State.SUBWOOFER,
        new ParallelCommandGroup(
            new RunCommand(() -> setFlywheelTarget(SUBWOOFER_SHOT)),
            atSpeedCommand(() -> SUBWOOFER_SHOT, SUBWOOFER_TOLERANCE)));

    registerStateCommand(State.IDLE, this::stop);
    registerStateCommand(State.TRAP, () -> setFlywheelTarget(TRAP_SHOT));
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.AMP);
    addOmniTransition(State.SPEAKER);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.SUBWOOFER);
  }

  public void setFlywheelTarget(Measure<Velocity<Angle>> velocity) {
    leader.setControl(request.withVelocity(velocity));
  }

  private Command atSpeedCommand(
      Supplier<Measure<Velocity<Angle>>> speedProvider, Measure<Velocity<Angle>> accuracy) {
    return new RunCommand(
        () -> {
          if (leader.getVelocity().getValue().minus(speedProvider.get()).magnitude()
              < accuracy.magnitude()) {
            setFlag(State.AT_SPEED);
          } else {
            clearFlag(State.AT_SPEED);
          }
        });
  }

  public void stop() {
    leader.stopMotor();
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    SPEAKER,
    AMP,
    TRAP,
    SUBWOOFER,

    // flags
    AT_SPEED
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }
}
