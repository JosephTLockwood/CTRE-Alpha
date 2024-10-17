// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.SwerveRequests;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.utils.statemachine.StateMachine;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final LineBreak lineBreak = new LineBreak();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Arm arm = new Arm(() -> Meters.of(1));
  private final Magazine magazine = new Magazine();
  private final Flywheel flywheel =
      new Flywheel(() -> RotationsPerSecond.of(0)); // TODO: Actually calculate distance to speaker

  private final SwerveRequests.FieldCentricSwerveSetpoint drive =
      new SwerveRequests.FieldCentricSwerveSetpoint(drivetrain::getState)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withDeadband(TunerConstants.kSpeedAt12Volts.times(0.1))
          .withRotationalDeadband(
              TunerConstants.kRotationAt12Volts.times(
                  0.1)); // Add a 10% deadband based on the max speed
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        TunerConstants.kSpeedAt12Volts.times(
                            -joystick.getLeftY())) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        TunerConstants.kSpeedAt12Volts.times(
                            -joystick.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        TunerConstants.kRotationAt12Volts.times(-joystick.getRightX()))
            // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick
        .rightBumper()
        .whileTrue(
            drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0))));
    joystick.start().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d());
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    super("RobotContainer", State.UNDETERMINED, State.class);
    registerStateCommands();
    registerStateTransitions();
    configureBindings();
    configureNamedCommands();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "intake", intake.transitionCommand(Intake.State.INTAKE_DOWN, false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void registerStateCommands() {

    Command feedIfArmInIntakePosition =
        Commands.either(
            magazine.transitionCommand(Magazine.State.FEEDING),
            magazine.transitionCommand(Magazine.State.IDLE),
            () ->
                arm.isFlag(Arm.State.ARM_AT_POSITION)
                    && lineBreak.getState() == LineBreak.State.HAS_GAME_PIECE);

    registerStateCommand(
        State.SOFT_E_STOP,
        Commands.parallel(
            climber.transitionCommand(Climber.State.SOFT_E_STOP),
            intake.transitionCommand(Intake.State.SOFT_E_STOP),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            arm.transitionCommand(Arm.State.IDLE),
            magazine.transitionCommand(Magazine.State.IDLE)));

    registerStateCommand(
        State.GROUND_INTAKE,
        Commands.parallel(
            intake.transitionCommand(Intake.State.INTAKE_DOWN),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            arm.transitionCommand(Arm.State.INTAKE),
            Commands.either(
                magazine.transitionCommand(Magazine.State.FEEDING),
                magazine.transitionCommand(Magazine.State.IDLE),
                () ->
                    arm.isFlag(Arm.State.ARM_AT_POSITION)
                        && arm.isFlag(Arm.State.WRIST_AT_POSITION)
                        && lineBreak.getState() != LineBreak.State.LOADED)));

    registerStateCommand(
        State.TRAVERSING,
        Commands.parallel(
            intake.transitionCommand(Intake.State.INTAKE_UP),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            arm.transitionCommand(Arm.State.INTAKE),
            feedIfArmInIntakePosition));

    registerStateCommand(
        State.SOURCE_INTAKE,
        Commands.parallel(
            intake.transitionCommand(Intake.State.INTAKE_UP),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            arm.transitionCommand(Arm.State.SOURCE_INTAKE),
            magazine.transitionCommand(Magazine.State.IDLE)));

    registerStateCommand(
        State.SHOOTING,
        Commands.parallel(
            flywheel.transitionCommand(Flywheel.State.SPEAKER),
            arm.transitionCommand(Arm.State.SPEAKER),
            feedIfArmInIntakePosition));

    registerStateCommand(
        State.AMP,
        Commands.either(
            Commands.parallel(
                flywheel.transitionCommand(Flywheel.State.AMP),
                arm.transitionCommand(Arm.State.AMP),
                magazine.transitionCommand(Magazine.State.IDLE)),
            Commands.parallel(
                flywheel.transitionCommand(Flywheel.State.IDLE),
                arm.transitionCommand(Arm.State.INTAKE),
                feedIfArmInIntakePosition),
            () -> lineBreak.getState() == LineBreak.State.LOADED));

    registerStateCommand(
        State.CLIMB,
        Commands.parallel(
            Commands.either(
                Commands.parallel(
                    climber.transitionCommand(Climber.State.TRAP_CLIMB),
                    arm.transitionCommand(Arm.State.TRAP)),
                Commands.parallel(
                    climber.transitionCommand(Climber.State.QUICK_CLIMB),
                    arm.transitionCommand(Arm.State.INTAKE)),
                () -> lineBreak.getState() == LineBreak.State.LOADED),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            intake.transitionCommand(Intake.State.INTAKE_UP)));
  }

  private void registerStateTransitions() {
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.AUTONOMOUS);
    addOmniTransition(State.GROUND_INTAKE);
    addOmniTransition(State.SOURCE_INTAKE);
    addOmniTransition(State.TRAVERSING);
    addOmniTransition(State.SHOOTING);
    addOmniTransition(State.CLIMB);
    addOmniTransition(State.AMP);
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    AUTONOMOUS,
    GROUND_INTAKE,
    SOURCE_INTAKE,
    TRAVERSING,
    SHOOTING,
    CLIMB,
    AMP
  }
}
