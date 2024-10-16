package frc.robot.subsystems.vision;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/** Represents a vision provider. */
public abstract class VisionProvider {
  protected String[] cameraNames;
  protected Supplier<SwerveDriveState> swerveStateSupplier;
  protected ArrayList<Pair<PoseEstimate, Vector<N3>>> visionMeasurements = new ArrayList<>();

  /**
   * Constructs a VisionProvider object with the specified camera name and swerve state supplier.
   *
   * @param cameraName the name of the camera
   * @param swerveStateSupplier the supplier for the swerve drive state
   */
  protected VisionProvider(String[] cameraNames, Supplier<SwerveDriveState> swerveStateSupplier) {
    this.cameraNames = cameraNames;
    this.swerveStateSupplier = swerveStateSupplier;
  }

  public List<Pair<PoseEstimate, Vector<N3>>> updateVisionMeasurements() {
    visionMeasurements.clear();
    for (String cameraName : cameraNames) {
      String signalPath = "Odometry/" + cameraName;
      PoseEstimate[] mtArray = getVisionUpdate(cameraName);
      PoseEstimate mt1 = mtArray[0];
      PoseEstimate mt2 = mtArray[1];
      mt1 = adjustPoseEstimate(signalPath + "/MT1/", mt1);
      mt2 = adjustPoseEstimate(signalPath + "/MT2/", mt2);
      PoseEstimate mt = filterPoseEstimate(mt1, mt2);
      writePoseEstimate(signalPath, mt);
      if (Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(mt))) {
        visionMeasurements.add(getVisionMeasurement(mt));
      }
    }
    // Sort by timestamp
    return visionMeasurements.stream()
        .sorted(
            (a, b) -> Double.compare(a.getFirst().timestampSeconds, b.getFirst().timestampSeconds))
        .toList();
  }

  /**
   * Retrieves the vision update.
   *
   * @return the pose estimate representing the vision update Note: This method is abstract and must
   *     be implemented by a subclass.
   */
  protected PoseEstimate[] getVisionUpdate(String cameraName) {
    return new PoseEstimate[] {new PoseEstimate(), new PoseEstimate()};
  }

  /**
   * Filters the pose estimate.
   *
   * @param mt1 The first pose estimate
   * @param mt2 The second pose estimate
   * @param swerveStateSupplier The supplier for the swerve drive state
   * @return The filtered pose estimate
   */
  protected PoseEstimate filterPoseEstimate(PoseEstimate mt1, PoseEstimate mt2) {
    PoseEstimate mt = DriverStation.isEnabled() ? mt1 : mt2;
    // If our angular velocity is greater than 80 degrees per second
    if (Math.abs(this.swerveStateSupplier.get().Speeds.omegaRadiansPerSecond)
            > Units.degreesToRadians(80)
        || Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return new PoseEstimate();
    }
    return mt;
  }

  private PoseEstimate adjustPoseEstimate(String signalPath, PoseEstimate poseEstimate) {
    double ntLatency = NetworkTablesJNI.now() * 1.0e-6 - poseEstimate.timestampSeconds;
    poseEstimate.timestampSeconds =
        Utils.getCurrentTimeSeconds() - (ntLatency + poseEstimate.latency);
    writePoseEstimate(signalPath, poseEstimate);
    return poseEstimate;
  }

  private void writePoseEstimate(String signalPath, PoseEstimate poseEstimate) {
    if (Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(poseEstimate))) {
      SignalLogger.writeDoubleArray(
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
      long[] tagIds = Arrays.stream(poseEstimate.rawFiducials).mapToLong(id -> id.id).toArray();
      SignalLogger.writeIntegerArray(signalPath + "/Tags/", tagIds);
      SignalLogger.writeBoolean(signalPath + "/Valid/", true);
    } else {
      SignalLogger.writeBoolean(signalPath + "/Valid/", false);
    }
  }

  /**
   * Retrieves the vision measurement.
   *
   * @param cameraName the name of the camera
   * @param mt the pose estimate
   * @return the vision measurement
   */
  public Pair<PoseEstimate, Vector<N3>> getVisionMeasurement(PoseEstimate mt) {
    // This check should not be necessary (current code only adds valid poses reach here),
    // To ensure that the all poses are valid with other implementations
    if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return new Pair<>(mt, VecBuilder.fill(0.0, 0.0, 0.0));
    }
    double xyStdDev = calculateXYStdDev(mt);
    double thetaStdDev = mt.isMegaTag2 ? 9999999 : calculateThetaStdDev(mt);
    return new Pair<>(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
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
}
