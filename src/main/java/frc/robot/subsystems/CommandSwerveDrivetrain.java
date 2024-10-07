package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforward;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private final Thread limelightThread =
      new Thread(
          new Limelight(
              new String[] {"limelight-fl", "limelight-fr", "limelight-bl", "limelight-br"},
              this::addVisionMeasurement,
              this::getState));
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private static final double DEADBAND = 0.1;
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          TunerConstants.robotConfig, TunerConstants.maxSteerVelocityRadsPerSec);
  private SwerveSetpoint previousSetpoint;

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(RotationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so user should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    super(drivetrainConstants, modules);
    configurePathPlanner();
    configureVision();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so user should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(drivetrainConstants, OdometryUpdateFrequency, modules);
    configurePathPlanner();
    configureVision();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so user should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation
   * @param visionStandardDeviation The standard deviation for vision calculation
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    configurePathPlanner();
    configureVision();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configure(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds, feedforward) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(10, 0, 0)),
        TunerConstants.robotConfig,
        () ->
            DriverStation.getAlliance().orElse(Alliance.Blue)
                == Alliance.Red, // Assume the path needs to be
        // flipped for Red vs Blue,
        // this is normally
        // the case
        this); // Subsystem for requirements

    ChassisSpeeds currentSpeeds =
        this.getState().Speeds; // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates =
        this.getState().ModuleStates; // Method to get the current swerve module states
    DriveFeedforward[] currentFeedforwards =
        new DriveFeedforward
            [TunerConstants.robotConfig.numModules]; // Method to get the current feedforwards
    Arrays.fill(currentFeedforwards, new DriveFeedforward(0, 0, 0));
    previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, currentFeedforwards);
  }

  public void configureVision() {
    limelightThread.setName("LimeLight Thread");
    limelightThread.setDaemon(true);
    limelightThread.start();
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(this.m_modules)
        .mapToDouble(module -> module.getDriveMotor().getPosition().getValueAsDouble())
        .toArray();
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command joystickDrive(
      SwerveRequest.ApplyChassisSpeeds drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          double robotRelativeXVel =
              linearVelocity.getX() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
          double robotRelativeYVel =
              linearVelocity.getY() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
          double robotRelativeOmega = omega * TunerConstants.kRotationAt12Volts.baseUnitMagnitude();

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  robotRelativeXVel,
                  robotRelativeYVel,
                  robotRelativeOmega,
                  isFlipped
                      ? this.getState().Pose.getRotation().plus(new Rotation2d(Math.PI))
                      : this.getState().Pose.getRotation());

          // Convert to field relative speeds & send command
          // Working
          // this.setControl(drive.withSpeeds(chassisSpeeds));
          // Not Working
          ChassisSpeeds setPointSpeeds = setPointGenerator(chassisSpeeds);
          System.out.println(setPointSpeeds.toString());
          this.setControl(drive.withSpeeds(setPointGenerator(setPointSpeeds)));
        },
        this);
  }

  /**
   * This method will take in desired robot-relative chassis speeds, generate a swerve setpoint,
   * then set the target state for each module
   *
   * @param speeds The desired robot-relative speeds
   */
  public ChassisSpeeds setPointGenerator(ChassisSpeeds speeds) {
    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );
    return previousSetpoint.robotRelativeSpeeds();
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    /*
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     */
    /*
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
