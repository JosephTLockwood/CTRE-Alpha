package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.RobotMode;
import frc.robot.RobotMode.Mode;
import frc.robot.utils.SignalHandler;
import java.util.Arrays;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Limelight implements Runnable {
  /**
   * Constructs a Limelight interface with the given limelight names.
   *
   * @param limelights Drivetrain-wide constants for the swerve drive
   * @param poseConsumer The consumer of the vision measurements
   * @param swerveStateSupplier The supplier of the swerve drive state
   */
  private final String[] limelights;

  private final VisionMeasurement poseConsumer;
  private final Supplier<SwerveDriveState> swerveStateSupplier;

  public Limelight(
      String[] limelights,
      VisionMeasurement poseConsumer,
      Supplier<SwerveDriveState> swerveStateSupplier) {
    this.limelights = limelights;
    this.poseConsumer = poseConsumer;
    this.swerveStateSupplier = swerveStateSupplier;
    SignalLogger.start();
  }

  @Override
  public void run() {
    if (RobotMode.getMode() == Mode.REPLAY) {
      HootReplay.waitForPlaying(10);
    }
    while (!Thread.interrupted()) {
      updateVisionMeasurements();
    }
  }

  /** Update the vision measurements. */
  private void updateVisionMeasurements() {
    for (String cameraName : limelights) {
      PoseEstimate mt = getVisionUpdate(cameraName);
      Pair<PoseEstimate, Vector<N3>> visionMeasurement =
          VisionHelper.getVisionMeasurement(cameraName, mt);
      if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(visionMeasurement.getFirst()))) {
        continue;
      }
      poseConsumer.addVisionMeasurement(
          visionMeasurement.getFirst().pose,
          Utils.getCurrentTimeSeconds() - visionMeasurement.getFirst().latency,
          visionMeasurement.getSecond());
    }
  }

  private PoseEstimate getVisionUpdate(String limelightName) {
    PoseEstimate mt1;
    PoseEstimate mt2;
    if (RobotMode.getMode() == Mode.REPLAY) {
      mt1 = readPoseEstimate("Odometry/MT1/" + limelightName);
      mt2 = readPoseEstimate("Odometry/MT2/" + limelightName);
    } else {
      LimelightHelpers.SetRobotOrientation(
          limelightName, swerveStateSupplier.get().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      VisionHelper.writePoseEstimate("Odometry/MT1/" + limelightName, mt1);
      VisionHelper.writePoseEstimate("Odometry/MT2/" + limelightName, mt2);
    }

    return VisionHelper.filterPoseEstimate(mt1, mt2, swerveStateSupplier);
  }

  private PoseEstimate readPoseEstimate(String signalPath) {
    SignalData<Boolean> validPoseEstimate =
        SignalHandler.readValue(signalPath + "/Valid/", Boolean.FALSE);
    if (validPoseEstimate.status != StatusCode.OK
        || Boolean.FALSE.equals(validPoseEstimate.value)) {
      return new PoseEstimate();
    }
    PoseEstimate poseEstimate =
        readPoseEstimateFromSignal(SignalHandler.readValue(signalPath, new double[] {}));
    RawFiducial[] rawFiducials =
        getFiducialsFromSignal(SignalHandler.readValue(signalPath + "/Tags/", new long[] {}));
    poseEstimate.rawFiducials = rawFiducials;
    return poseEstimate;
  }

  private PoseEstimate readPoseEstimateFromSignal(SignalData<double[]> signalData) {
    if (signalData.status != StatusCode.OK || signalData.value.length != 8) {
      return new PoseEstimate();
    }
    double[] data = signalData.value;
    return new PoseEstimate(
        new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[2])),
        data[3],
        data[4],
        (int) data[5],
        0.0,
        data[6],
        0.0,
        new RawFiducial[] {},
        data[7] == 1);
  }

  private RawFiducial[] getFiducialsFromSignal(SignalData<long[]> signalData) {
    if (signalData.status != StatusCode.OK) {
      return new RawFiducial[] {};
    }
    return Arrays.stream(signalData.value)
        .mapToObj(id -> new RawFiducial((int) id, 0, 0, 0, 0, 0, 0))
        .toArray(RawFiducial[]::new);
  }

  @FunctionalInterface
  public interface VisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
