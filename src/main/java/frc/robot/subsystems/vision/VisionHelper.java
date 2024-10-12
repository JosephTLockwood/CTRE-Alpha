package frc.robot.subsystems.vision;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utils.SignalHandler;
import java.util.Arrays;
import java.util.function.Supplier;

public final class VisionHelper {

  private VisionHelper() {}

  public static void writePoseEstimate(String signalPath, PoseEstimate poseEstimate) {
    if (Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(poseEstimate))) {
      SignalHandler.writeValue(
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
      SignalHandler.writeValue(signalPath + "/Tags/", tagIds);
      SignalLogger.writeBoolean(signalPath + "/Valid/", true);
      return;
    }
    SignalLogger.writeBoolean(signalPath + "/Valid/", false);
  }

  public static PoseEstimate filterPoseEstimate(
      PoseEstimate mt1, PoseEstimate mt2, Supplier<SwerveDriveState> swerveStateSupplier) {
    PoseEstimate mt = DriverStation.isEnabled() ? mt1 : mt2;
    // If our angular velocity is greater than 80 degrees per second
    if (Math.abs(swerveStateSupplier.get().Speeds.omegaRadiansPerSecond)
            > Units.degreesToRadians(80)
        || Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return new PoseEstimate();
    }
    return mt;
  }
}
