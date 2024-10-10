package frc.robot.subsystems;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.Comparator;
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
   */
  private final String[] limelights;

  private final LimelightVisionMeasurement poseConsumer;
  private final Supplier<SwerveDriveState> swerveStateSupplier;

  private ArrayList<Pair<PoseEstimate, Vector<N3>>> poseEstimates;

  public Limelight(
      String[] limelights,
      LimelightVisionMeasurement poseConsumer,
      Supplier<SwerveDriveState> swerveStateSupplier) {
    this.limelights = limelights;
    this.poseConsumer = poseConsumer;
    this.swerveStateSupplier = swerveStateSupplier;
  }

  @Override
  public void run() {
    while (!Thread.interrupted()) {
      poseEstimates.clear();
      updateVisionMeasurements();
    }
  }

  /** Update the vision measurements. */
  private void updateVisionMeasurements() {
    for (String limelightName : limelights) {
      PoseEstimate mt = getVisionUpdate(limelightName);
      double xyStdDev = calculateXYStdDev(mt);
      double thetaStdDev = mt.isMegaTag2 ? 9999999 : calculateThetaStdDev(mt);
      SignalLogger.writeDoubleArray(
          "Odometry/" + limelightName,
          new double[] {mt.pose.getX(), mt.pose.getY(), mt.pose.getRotation().getDegrees()});
      poseEstimates.add(new Pair<>(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
    }
    // Sort poseEstimates and send to consumer
    poseEstimates.stream()
        .sorted(Comparator.comparingDouble(pair -> pair.getFirst().timestampSeconds))
        .forEach(
            pair ->
                poseConsumer.addVisionMeasurement(
                    pair.getFirst().pose,
                    pair.getFirst().timestampSeconds - pair.getFirst().latency,
                    pair.getSecond()));
  }

  private PoseEstimate getVisionUpdate(String limelightName) {
    LimelightHelpers.SetRobotOrientation(
        limelightName, swerveStateSupplier.get().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate mt1;
    PoseEstimate mt2;
    if (HootReplay.isPlaying()) {
      SignalData<double[]> mt1Array = HootReplay.getDoubleArray("Odometry/MT1/" + limelightName);
      SignalData<double[]> mt2Array = HootReplay.getDoubleArray("Odometry/MT2/" + limelightName);
      if (mt1Array.status != StatusCode.OK || mt2Array.status != StatusCode.OK) {
        Thread.currentThread().interrupt();
      }
      mt1 =
          new PoseEstimate(
              new Pose2d(
                  mt1Array.value[0], mt1Array.value[1], Rotation2d.fromDegrees(mt1Array.value[2])),
              mt1Array.value[3],
              mt1Array.value[4],
              (int) mt1Array.value[5],
              0.0,
              mt1Array.value[6],
              0.0,
              new RawFiducial[] {},
              mt1Array.value[7] == 1);
      mt2 =
          new PoseEstimate(
              new Pose2d(
                  mt2Array.value[0], mt2Array.value[1], Rotation2d.fromDegrees(mt2Array.value[2])),
              mt2Array.value[3],
              mt2Array.value[4],
              (int) mt2Array.value[5],
              0.0,
              mt2Array.value[6],
              0.0,
              new RawFiducial[] {},
              mt2Array.value[7] == 1);
    } else {
      mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    PoseEstimate mt = DriverStation.isEnabled() ? mt1 : mt2;
    // If our angular velocity is greater than 80 degrees per second, ignore vision updates
    if (Math.abs(swerveStateSupplier.get().Speeds.omegaRadiansPerSecond)
            > Units.degreesToRadians(80)
        || Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      Thread.currentThread().interrupt();
    }
    if (Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(mt1))) {
      SignalLogger.writeDoubleArray(
          "Odometry/MT1/" + limelightName,
          new double[] {
            mt1.pose.getX(),
            mt1.pose.getY(),
            mt1.pose.getRotation().getDegrees(),
            mt1.timestampSeconds,
            mt1.latency,
            mt1.tagCount,
            mt1.avgTagDist,
            mt1.isMegaTag2 ? 1 : 0
          });
    }
    if (Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(mt2))) {
      SignalLogger.writeDoubleArray(
          "Odometry/MT2/" + limelightName,
          new double[] {
            mt2.pose.getX(),
            mt2.pose.getY(),
            mt2.pose.getRotation().getDegrees(),
            mt2.timestampSeconds,
            mt2.latency,
            mt2.tagCount,
            mt2.avgTagDist,
            mt2.isMegaTag2 ? 1 : 0
          });
    }
    return mt;
  }

  /**
   * Calculate the standard deviation of the x and y coordinates.
   *
   * @param avgTagDist The pose estimate
   * @param tagPosesSize The number of detected tag poses
   * @return The standard deviation of the x and y coordinates
   */
  private double calculateXYStdDev(PoseEstimate mt) {
    return TunerConstants.visionStandardDeviationXY * Math.pow(mt.avgTagDist, 2.0) / mt.tagCount;
  }

  /**
   * Calculate the standard deviation of the theta coordinate.
   *
   * @param avgTagDist The pose estimate
   * @param tagPosesSize The number of detected tag poses
   * @return The standard deviation of the theta coordinate
   */
  private double calculateThetaStdDev(PoseEstimate mt) {
    return TunerConstants.visionStandardDeviationTheta * Math.pow(mt.avgTagDist, 2.0) / mt.tagCount;
  }

  @FunctionalInterface
  public interface LimelightVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
