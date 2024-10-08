// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.SwerveRequests;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SwerveRequests.ApplyChassisSetpoint drive =
      new SwerveRequests.ApplyChassisSetpoint(drivetrain::getState)
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
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
