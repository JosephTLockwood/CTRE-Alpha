package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.12).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final Measure<Current> kSlipCurrent = Amps.of(120.0);

  // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = null;

  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final Measure<Velocity<Distance>> kSpeedAt12Volts = MetersPerSecond.of(4.830);
  public static final Measure<Velocity<Angle>> kRotationAt12Volts = RotationsPerSecond.of(0.75);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 0;

  public static final double kDriveGearRatio = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double kSteerGearRatio = 150.0 / 7.0;

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  //   private static final CANBus kCANBus = new CANBus("canivore");
  private static final CANBus kCANBus =
      new CANBus("canivore", "./logs/sim_2024-10-14_00-26-02.hoot");

  private static final int kPigeonId = 13;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  // Simulated voltage necessary to overcome friction
  private static final Measure<Voltage> kSteerFrictionVoltage = Volts.of(0.25);
  private static final Measure<Voltage> kDriveFrictionVoltage = Volts.of(0.25);

  // Vision and Odometry Standard Deviations
  public static final Matrix<N3, N1> odometryStandardDeviation =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
  public static final double visionStandardDeviationXY = 0.01;
  public static final double visionStandardDeviationTheta = 0.01;

  private static Measure<Mass> robotWeight = Pounds.of(125);
  private static double robotMOI = 12.34;
  private static Measure<Distance> trackwidth = Inches.of(22.75);
  private static Measure<Distance> wheelbase = Inches.of(22.75);
  private static Measure<Distance> kWheelRadius = Inches.of(2.0);
  public static Measure<Velocity<Angle>> maxSteerVelocityRadsPerSec = RotationsPerSecond.of(4.66);

  private static ModuleConfig moduleConfig =
      new ModuleConfig(
          kWheelRadius.in(Meters),
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          1.0,
          DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.kDriveGearRatio),
          80,
          4);
  public static final RobotConfig robotConfig =
      new RobotConfig(
          robotWeight.in(Kilogram),
          robotMOI,
          moduleConfig,
          trackwidth.in(Meters),
          wheelbase.in(Meters));

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio)
          .withWheelRadius(kWheelRadius)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 9;
  private static final Measure<Angle> kFrontLeftEncoderOffset = Rotations.of(0);
  private static final boolean kFrontLeftSteerMotorInverted = false;

  private static final Measure<Distance> kFrontLeftXPos = trackwidth.divide(2);
  private static final Measure<Distance> kFrontLeftYPos = wheelbase.divide(2);

  // Front Right
  private static final int kFrontRightDriveMotorId = 3;
  private static final int kFrontRightSteerMotorId = 4;
  private static final int kFrontRightEncoderId = 10;
  private static final Measure<Angle> kFrontRightEncoderOffset = Rotations.of(0);
  private static final boolean kFrontRightSteerMotorInverted = false;

  private static final Measure<Distance> kFrontRightXPos = trackwidth.divide(2);
  private static final Measure<Distance> kFrontRightYPos = wheelbase.divide(2).negate();

  // Back Left
  private static final int kBackLeftDriveMotorId = 5;
  private static final int kBackLeftSteerMotorId = 6;
  private static final int kBackLeftEncoderId = 11;
  private static final Measure<Angle> kBackLeftEncoderOffset = Rotations.of(0);
  private static final boolean kBackLeftSteerMotorInverted = false;

  private static final Measure<Distance> kBackLeftXPos = trackwidth.divide(2).negate();
  private static final Measure<Distance> kBackLeftYPos = wheelbase.divide(2);

  // Back Right
  private static final int kBackRightDriveMotorId = 7;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 12;
  private static final Measure<Angle> kBackRightEncoderOffset = Rotations.of(0);
  private static final boolean kBackRightSteerMotorInverted = false;

  private static final Measure<Distance> kBackRightXPos = trackwidth.divide(2).negate();
  private static final Measure<Distance> kBackRightYPos = wheelbase.divide(2).negate();

  public static final Measure<Distance> kRobotRadius =
      Meter.of(
          new Translation2d(kBackLeftXPos.baseUnitMagnitude(), kBackLeftYPos.baseUnitMagnitude())
                  .getDistance(
                      new Translation2d(
                          kFrontRightXPos.baseUnitMagnitude(), kFrontRightYPos.baseUnitMagnitude()))
              / 2);

  private static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftDriveMotorId,
          kFrontLeftSteerMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          kFrontLeftXPos,
          kFrontLeftYPos,
          kInvertLeftSide,
          kFrontLeftSteerMotorInverted);
  private static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightDriveMotorId,
          kFrontRightSteerMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          kFrontRightXPos,
          kFrontRightYPos,
          kInvertRightSide,
          kFrontRightSteerMotorInverted);
  private static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftDriveMotorId,
          kBackLeftSteerMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          kBackLeftXPos,
          kBackLeftYPos,
          kInvertLeftSide,
          kBackLeftSteerMotorInverted);
  private static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightDriveMotorId,
          kBackRightSteerMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          kBackRightXPos,
          kBackRightYPos,
          kInvertRightSide,
          kBackRightSteerMotorInverted);

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  public static CommandSwerveDrivetrain createDrivetrain() {
    return new CommandSwerveDrivetrain(
        DrivetrainConstants,
        250,
        odometryStandardDeviation,
        VecBuilder.fill(
            visionStandardDeviationXY, visionStandardDeviationXY, visionStandardDeviationTheta),
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight);
  }
}
