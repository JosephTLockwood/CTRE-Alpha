package frc.robot.subsystems.vision;

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
import frc.robot.utils.SignalHandler;
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

  private final VisionMeasurement poseConsumer;
  private final Supplier<SwerveDriveState> swerveStateSupplier;

  private ArrayList<Pair<PoseEstimate, Vector<N3>>> poseEstimates = new ArrayList<>();

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
    while (!Thread.interrupted()) {
      poseEstimates.clear();
      updateVisionMeasurements();
    }
  }

  /** Update the vision measurements. */
  private void updateVisionMeasurements() {
    for (String limelightName : limelights) {
      PoseEstimate mt = getVisionUpdate(limelightName);
      if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
        continue;
      }
      double xyStdDev = calculateXYStdDev(mt);
      double thetaStdDev = mt.isMegaTag2 ? 9999999 : calculateThetaStdDev(mt);
      SignalHandler.getOrWriteSignal(
          "Odometry/" + limelightName,
          new double[] {mt.pose.getX(), mt.pose.getY(), mt.pose.getRotation().getDegrees()});
      poseEstimates.add(new Pair<>(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
    }
    // Sort poseEstimates and send to consumer
    poseEstimates.stream()
        .sorted(
            Comparator.comparingDouble(
                pair -> pair.getFirst().timestampSeconds - pair.getFirst().latency))
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
    PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    SignalData<double[]> mt1Signal = getOrWritePoseEstimate("Odometry/MT1/" + limelightName, mt1);
    SignalData<double[]> mt2Signal = getOrWritePoseEstimate("Odometry/MT2/" + limelightName, mt2);

    PoseEstimate mt =
        DriverStation.isEnabled()
            ? getPoseEstimateFromSignal(mt1Signal)
            : getPoseEstimateFromSignal(mt2Signal);

    // If our angular velocity is greater than 80 degrees per second, or if the pose estimate is
    // invalid, interrupt thread
    if (Math.abs(swerveStateSupplier.get().Speeds.omegaRadiansPerSecond)
            > Units.degreesToRadians(80)
        || Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return new PoseEstimate();
    }

    return mt;
  }

  private SignalData<double[]> getOrWritePoseEstimate(
      String signalPath, PoseEstimate poseEstimate) {
    if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(poseEstimate))) {
      SignalData<double[]> signalData = new SignalData<>();
      signalData.status = StatusCode.NotFound;
      return new SignalData<>();
    }
    return SignalHandler.getOrWriteSignal(
        signalPath,
        new double[] {
          poseEstimate.pose.getX(),
          poseEstimate.pose.getY(),
          poseEstimate.pose.getRotation().getDegrees(),
          poseEstimate.timestampSeconds,
          poseEstimate.latency,
          poseEstimate.tagCount,
          poseEstimate.avgTagDist,
          poseEstimate.isMegaTag2 ? 1 : 0
        });
  }

  private PoseEstimate getPoseEstimateFromSignal(SignalData<double[]> signalData) {
    if (signalData.status != StatusCode.OK) {
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
  public interface VisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
