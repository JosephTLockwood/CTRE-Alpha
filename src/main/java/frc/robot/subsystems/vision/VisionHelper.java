package frc.robot.subsystems.vision;

import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.RobotMode;
import frc.robot.generated.TunerConstants;
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

  public static Pair<PoseEstimate, Vector<N3>> getVisionMeasurement(
      String cameraName, PoseEstimate mt) {
    writePoseEstimate("Odometry/" + cameraName, mt);
    if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return new Pair<>(mt, VecBuilder.fill(0.0, 0.0, 0.0));
    }
    double xyStdDev = calculateXYStdDev(mt);
    double thetaStdDev = mt.isMegaTag2 ? 9999999 : calculateThetaStdDev(mt);
    return new Pair<>(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  public static long getTimeDiffrence(String cameraName, double timestampSeconds) {
    String timeDifPath = "Odometry/" + cameraName + "/Time Diffrence/";
    if (RobotMode.getMode() == RobotMode.Mode.REPLAY) {
      return SignalHandler.readValue(timeDifPath, 0L).value;
    }
    return writeTimeDiffrence(timeDifPath, timestampSeconds);
  }

  private static long writeTimeDiffrence(String timeDifPath, double timestampSeconds) {
    long timeDiffrence = (long) (NetworkTablesJNI.now() * 1e-6 - timestampSeconds);
    SignalHandler.writeValue(timeDifPath, timeDiffrence);
    return timeDiffrence;
  }

  /**
   * Calculate the standard deviation of the x and y coordinates.
   *
   * @param avgTagDist The pose estimate
   * @param tagPosesSize The number of detected tag poses
   * @return The standard deviation of the x and y coordinates
   */
  private static double calculateXYStdDev(PoseEstimate mt) {
    return TunerConstants.visionStandardDeviationXY * Math.pow(mt.avgTagDist, 2.0) / mt.tagCount;
  }

  /**
   * Calculate the standard deviation of the theta coordinate.
   *
   * @param avgTagDist The pose estimate
   * @param tagPosesSize The number of detected tag poses
   * @return The standard deviation of the theta coordinate
   */
  private static double calculateThetaStdDev(PoseEstimate mt) {
    return TunerConstants.visionStandardDeviationTheta * Math.pow(mt.avgTagDist, 2.0) / mt.tagCount;
  }
}
