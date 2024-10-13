package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.StatusCode;
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

public class Limelight {

  private final String cameraName;
  private final Supplier<SwerveDriveState> swerveStateSupplier;

  public Limelight(String cameraName, Supplier<SwerveDriveState> swerveStateSupplier) {
    this.cameraName = cameraName;
    this.swerveStateSupplier = swerveStateSupplier;
  }

  /** Update the vision measurements. */
  public Pair<PoseEstimate, Vector<N3>> updateVisionMeasurements() {
    PoseEstimate mt = getVisionUpdate();
    return VisionHelper.getVisionMeasurement(cameraName, mt);
  }

  private PoseEstimate getVisionUpdate() {
    PoseEstimate mt1;
    PoseEstimate mt2;
    if (RobotMode.getMode() == Mode.REPLAY) {
      mt1 = readPoseEstimate("Odometry/MT1/" + cameraName);
      mt2 = readPoseEstimate("Odometry/MT2/" + cameraName);
    } else {
      LimelightHelpers.SetRobotOrientation(
          cameraName, swerveStateSupplier.get().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
      VisionHelper.writePoseEstimate("Odometry/MT1/" + cameraName, mt1);
      VisionHelper.writePoseEstimate("Odometry/MT2/" + cameraName, mt2);
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
