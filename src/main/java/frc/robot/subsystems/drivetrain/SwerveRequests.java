package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;

public class SwerveRequests implements SwerveRequest {

  /** Sets the swerve drive modules to point to a specified direction. */
  public static class FieldCentricSwerveSetpoint implements NativeSwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public double VelocityX = 0;

    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;

    /**
     * The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     */
    public double RotationalRate = 0;

    /** The allowable deadband of the request, in m/s. */
    public double Deadband = 0;

    /** The rotational deadband of the request, in radians per second. */
    public double RotationalDeadband = 0;

    /** The chassis speeds to apply to the drivetrain. */
    public ChassisSpeeds Speeds = new ChassisSpeeds();

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;

    /** The type of control request to use for the drive motor. */
    public SwerveModule.SteerRequestType SteerRequestType =
        SwerveModule.SteerRequestType.MotionMagicExpo;

    private final SwerveSetpointGenerator setpointGenerator =
        new SwerveSetpointGenerator(
            TunerConstants.robotConfig,
            TunerConstants.maxSteerVelocityRadsPerSec.in(RadiansPerSecond));
    private SwerveSetpoint previousSetpoint;
    private Supplier<SwerveDriveState> state;

    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = false;

    public FieldCentricSwerveSetpoint(Supplier<SwerveDriveState> state) {
      this.state = state;
      ChassisSpeeds currentSpeeds =
          state.get().Speeds; // Method to get current robot-relative chassis speeds
      SwerveModuleState[] currentStates =
          state.get().ModuleStates; // Method to get the current swerve module states
      DriveFeedforwards currentFeedforwards =
          DriveFeedforwards.zeros(TunerConstants.robotConfig.numModules);
      previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, currentFeedforwards);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
      return StatusCode.OK;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     *
     * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withVelocityX(double newVelocityX) {
      this.VelocityX = newVelocityX;
      return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     *
     * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withVelocityX(LinearVelocity newVelocityX) {
      this.VelocityX = newVelocityX.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     *
     * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withVelocityY(double newVelocityY) {
      this.VelocityY = newVelocityY;
      return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     *
     * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withVelocityY(LinearVelocity newVelocityY) {
      this.VelocityY = newVelocityY.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the RotationalRate parameter and returns itself.
     *
     * <p>The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     *
     * @param newRotationalRate Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withRotationalRate(double newRotationalRate) {
      this.RotationalRate = newRotationalRate;
      return this;
    }

    /**
     * Modifies the RotationalRate parameter and returns itself.
     *
     * <p>The angular rate to rotate at, in radians per second. Angular rate is defined as
     * counterclockwise positive, so this determines how fast to turn counterclockwise.
     *
     * @param newRotationalRate Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withRotationalRate(AngularVelocity newRotationalRate) {
      this.RotationalRate = newRotationalRate.in(RadiansPerSecond);
      return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withDeadband(double newDeadband) {
      this.Deadband = newDeadband;
      return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withDeadband(LinearVelocity newDeadband) {
      this.Deadband = newDeadband.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withRotationalDeadband(double newRotationalDeadband) {
      this.RotationalDeadband = newRotationalDeadband;
      return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withRotationalDeadband(
        AngularVelocity newRotationalDeadband) {
      this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
      return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withDriveRequestType(
        SwerveModule.DriveRequestType newDriveRequestType) {
      this.DriveRequestType = newDriveRequestType;
      return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withSteerRequestType(
        SwerveModule.SteerRequestType newSteerRequestType) {
      this.SteerRequestType = newSteerRequestType;
      return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     *
     * <p>Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public FieldCentricSwerveSetpoint withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
      this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
      return this;
    }

    public void applyNative(int id) {
      SwerveSetpoint setPoint =
          setSwerveSpeed(
              this.VelocityX,
              this.VelocityY,
              this.RotationalRate,
              this.Deadband,
              this.RotationalDeadband);
      SwerveJNI.JNI_SetControl_ApplyChassisSpeeds(
          id,
          setPoint.robotRelativeSpeeds().vxMetersPerSecond,
          setPoint.robotRelativeSpeeds().vyMetersPerSecond,
          setPoint.robotRelativeSpeeds().omegaRadiansPerSecond,
          setPoint.feedforwards().robotRelativeForcesXNewtons(),
          setPoint.feedforwards().robotRelativeForcesYNewtons(),
          CenterOfRotation.getX(),
          CenterOfRotation.getY(),
          DriveRequestType.value,
          SteerRequestType.value,
          false);
    }

    private SwerveSetpoint setSwerveSpeed(
        double VelocityX,
        double VelocityY,
        double RotationalRate,
        double Deadband,
        double RotationalDeadband) {
      double linearMagnitude = MathUtil.applyDeadband(Math.hypot(VelocityX, VelocityY), Deadband);
      Rotation2d linearDirection = new Rotation2d(VelocityX, VelocityY);
      double omega = MathUtil.applyDeadband(RotationalRate, RotationalDeadband);

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

      LinearVelocity robotRelativeXVel =
          TunerConstants.kSpeedAt12Volts.times(linearVelocity.getX());
      LinearVelocity robotRelativeYVel =
          TunerConstants.kSpeedAt12Volts.times(linearVelocity.getY());
      AngularVelocity robotRelativeOmega = TunerConstants.kRotationAt12Volts.times(omega);
      ChassisSpeeds chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              robotRelativeXVel,
              robotRelativeYVel,
              robotRelativeOmega,
              isFlipped
                  ? state.get().Pose.getRotation().plus(new Rotation2d(Math.PI))
                  : state.get().Pose.getRotation());
      previousSetpoint =
          setpointGenerator.generateSetpoint(
              previousSetpoint, // The previous setpoint
              chassisSpeeds, // The desired target speeds
              0.02 // The loop time of the robot code, in seconds
              );
      return previousSetpoint;
    }
  }

  @Override
  public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
    return StatusCode.OK;
  }
}
