package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.utils.SignalHandler;
import java.util.Arrays;
import java.util.function.Supplier;

public class VisionReplay extends VisionProvider {

  /**
   * Constructs a VisionReplay object with the specified camera name and swerve state supplier.
   *
   * @param cameraName the name of the camera
   * @param swerveStateSupplier the supplier for the swerve drive state
   */
  public VisionReplay(String cameraName, Supplier<SwerveDriveState> swerveStateSupplier) {
    super(cameraName, swerveStateSupplier);
  }

  /**
   * Retrieves the vision update from the Limelight.
   *
   * @return the pose estimate representing the vision update
   */
  @Override
  protected PoseEstimate[] getVisionUpdate() {
    PoseEstimate mt1 = readPoseEstimate("Odometry/MT1/" + cameraName);
    PoseEstimate mt2 = readPoseEstimate("Odometry/MT2/" + cameraName);
    return new PoseEstimate[] {mt1, mt2};
  }

  /**
   * Reads a pose estimate from the specified signal path.
   *
   * @param signalPath the path to the signal
   * @return the pose estimate
   */
  private PoseEstimate readPoseEstimate(String signalPath) {
    SignalData<Boolean> validPoseEstimate =
        SignalHandler.readValue(signalPath + "/Valid/", Boolean.FALSE);
    if (validPoseEstimate.status != StatusCode.OK
        || Boolean.FALSE.equals(validPoseEstimate.value)) {
      return new PoseEstimate();
    }
    RawFiducial[] rawFiducials = getFiducialsFromSignal(signalPath);
    return readPoseEstimateFromSignal(signalPath, rawFiducials);
  }

  /**
   * Reads a pose estimate from the specified signal data.
   *
   * @param signalData the signal data
   * @return the pose estimate
   */
  private PoseEstimate readPoseEstimateFromSignal(String signalPath, RawFiducial[] rawFiducials) {
    SignalData<double[]> signalData = SignalHandler.readValue(signalPath, new double[] {});
    if (signalData.status != StatusCode.OK || signalData.value.length != 8) {
      return new PoseEstimate();
    }
    double[] data = signalData.value;
    // We may want this to act like a new vision reading using NetworkTablesJNI.now() maybe viable
    // (didn't seem to work)
    return new PoseEstimate(
        new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[2])),
        data[3],
        data[4],
        (int) data[5],
        0.0,
        data[6],
        0.0,
        rawFiducials,
        data[7] == 1);
  }

  /**
   * Retrieves the fiducials from the specified signal data.
   *
   * @param signalData the signal data
   * @return the fiducials
   */
  private RawFiducial[] getFiducialsFromSignal(String signalPath) {
    SignalData<long[]> signalData = SignalHandler.readValue(signalPath + "/Tags/", new long[] {});
    if (signalData.status != StatusCode.OK) {
      return new RawFiducial[] {};
    }
    return Arrays.stream(signalData.value)
        .mapToObj(id -> new RawFiducial((int) id, 0, 0, 0, 0, 0, 0))
        .toArray(RawFiducial[]::new);
  }
}
