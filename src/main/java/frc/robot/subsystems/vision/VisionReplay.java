package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class VisionReplay extends VisionProvider {

  /**
   * Constructs a VisionReplay object with the specified camera name and swerve state supplier.
   *
   * @param cameraNames the name of the camera
   * @param swerveStateSupplier the supplier for the swerve drive state
   */
  public VisionReplay(String[] cameraNames, Supplier<SwerveDriveState> swerveStateSupplier) {
    super(cameraNames, swerveStateSupplier);
  }

  @Override
  public List<Pair<PoseEstimate, Vector<N3>>> updateVisionMeasurements() {
    visionMeasurements.clear();
    for (String cameraName : cameraNames) {
      String signalPath = "Odometry/" + cameraName;
      SignalData<Boolean> validPoseEstimate = HootReplay.getBoolean(signalPath + "/Valid/");
      if (validPoseEstimate.status != StatusCode.OK
          || Boolean.FALSE.equals(validPoseEstimate.value)) {
        visionMeasurements.add(new Pair<>(new PoseEstimate(), VecBuilder.fill(0.0, 0.0, 0.0)));
      }
      RawFiducial[] rawFiducials = getFiducialsFromSignal(signalPath);
      PoseEstimate mt = readPoseEstimateFromSignal(rawFiducials, signalPath);
      visionMeasurements.add(getVisionMeasurement(mt));
    }
    return visionMeasurements;
  }

  /**
   * Reads a pose estimate from the specified signal data.
   *
   * @param signalData the signal data
   * @return the pose estimate
   */
  private PoseEstimate readPoseEstimateFromSignal(RawFiducial[] rawFiducials, String signalPath) {
    SignalData<double[]> signalData = HootReplay.getDoubleArray(signalPath);
    if (signalData.status != StatusCode.OK || signalData.value.length != 8) {
      return new PoseEstimate();
    }
    return new PoseEstimate(
        new Pose2d(
            signalData.value[0], signalData.value[1], Rotation2d.fromDegrees(signalData.value[2])),
        signalData.timestampSeconds - signalData.value[4],
        signalData.value[4],
        (int) signalData.value[5],
        0.0,
        signalData.value[6],
        0.0,
        rawFiducials,
        signalData.value[7] == 1);
  }

  /**
   * Retrieves the fiducials from the specified signal data.
   *
   * @param signalData the signal data
   * @return the fiducials
   */
  private RawFiducial[] getFiducialsFromSignal(String signalPath) {
    SignalData<long[]> signalData = HootReplay.getIntegerArray(signalPath + "/Tags/");
    if (signalData.status != StatusCode.OK) {
      return new RawFiducial[] {};
    }
    return Arrays.stream(signalData.value)
        .mapToObj(id -> new RawFiducial((int) id, 0, 0, 0, 0, 0, 0))
        .toArray(RawFiducial[]::new);
  }
}
