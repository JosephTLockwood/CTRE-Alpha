package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utils.SignalHandler;
import java.util.Arrays;
import java.util.function.Supplier;

public final class VisionHelper {
  public static void writePoseEstimate(String signalPath, PoseEstimate poseEstimate) {
    if (Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(poseEstimate))) {
      SignalHandler.getOrWriteSignal(
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
      SignalHandler.getOrWriteSignal(
          signalPath + "/Tags",
          Arrays.stream(poseEstimate.rawFiducials).mapToInt(id -> id.id).toArray());
    }
  }

  public static PoseEstimate filterPoseEstimate(
      PoseEstimate mt1, PoseEstimate mt2, Supplier<SwerveDriveState> swerveStateSupplier) {
    PoseEstimate mt = DriverStation.isEnabled() ? mt1 : mt2;
    // If our angular velocity is greater than 80 degrees per second, or if the pose estimate is
    // invalid, interrupt thread
    if (Math.abs(swerveStateSupplier.get().Speeds.omegaRadiansPerSecond)
            > Units.degreesToRadians(80)
        || Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return new PoseEstimate();
    }
    return mt;
  }
}
