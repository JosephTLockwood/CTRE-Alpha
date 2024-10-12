package frc.robot.subsystems.vision;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class PhotonVision implements Runnable {
  /**
   * Constructs a Limelight interface with the given limelight names.
   *
   * @param cameraName Drivetrain-wide constants for the swerve drive
   */
  private final VisionMeasurement poseConsumer;

  private final Supplier<SwerveDriveState> swerveStateSupplier;

  private final String cameraName;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  private double lastEstTimestamp = 0;

  private ArrayList<Pair<PoseEstimate, Vector<N3>>> poseEstimates = new ArrayList<>();

  public PhotonVision(
      String cameraName,
      Transform3d robotToCamera,
      VisionMeasurement poseConsumer,
      Supplier<SwerveDriveState> swerveStateSupplier) {
    this.cameraName = cameraName;
    camera = new PhotonCamera(cameraName);
    photonEstimator =
        new PhotonPoseEstimator(
            FieldConstants.aprilTags,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // Create the vision system simulation which handles cameras and targets on the
    // field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(FieldConstants.aprilTags);
    // Create simulated camera properties. These can be set to mimic your actual
    // camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    // with visible
    // targets.
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableDrawWireframe(true);
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
    PoseEstimate mt = getVisionUpdate(cameraName);
    Pair<PoseEstimate, Matrix<N3, N1>> visionMeasurement =
        VisionHelper.getVisionMeasurement(cameraName, mt);
    if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(visionMeasurement.getFirst()))) {
      return;
    }
    poseConsumer.addVisionMeasurement(
        visionMeasurement.getFirst().pose,
        Utils.getCurrentTimeSeconds() - visionMeasurement.getFirst().latency,
        visionMeasurement.getSecond());
  }

  private PoseEstimate getVisionUpdate(String cameraName) {
    visionSim.update(swerveStateSupplier.get().Pose);
    visionSim.getDebugField();
    PhotonPipelineResult results = cameraSim.getCamera().getLatestResult();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (results.targets.isEmpty() || allianceOptional.isEmpty()) {
      return new PoseEstimate();
    }
    double latencyMS = results.getLatencyMillis();
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
    if (estimatedPose.isPresent()) {
      Pose3d poseEstimation = estimatedPose.get().estimatedPose;
      double averageTagDistance = 0.0;
      long[] tagIDs =
          Arrays.stream(new int[results.targets.size()])
              .mapToLong(id -> results.targets.get(id).getFiducialId())
              .toArray();
      for (int i = 0; i < tagIDs.length; i++) {
        var tagPose = photonEstimator.getFieldTags().getTagPose((int) tagIDs[i]);
        if (tagPose.isEmpty()) {
          continue;
        }
        averageTagDistance +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(poseEstimation.getTranslation().toTranslation2d());
      }
      averageTagDistance /= tagIDs.length;

      PoseEstimate mt1 =
          createPoseEstimate(
              poseEstimation.toPose2d(), latencyMS, tagIDs, averageTagDistance, false);
      PoseEstimate mt2 =
          createPoseEstimate(
              poseEstimation.toPose2d(), latencyMS, tagIDs, averageTagDistance, true);

      VisionHelper.writePoseEstimate("Odometry/MT1/" + cameraName, mt1);
      VisionHelper.writePoseEstimate("Odometry/MT2/" + cameraName, mt2);

      return VisionHelper.filterPoseEstimate(mt1, mt2, swerveStateSupplier);
    }
    return new PoseEstimate();
  }

  private PoseEstimate createPoseEstimate(
      Pose2d pose, double latencyMS, long[] tagIds, double avgTagDist, boolean isMegaTag2) {
    RawFiducial[] rawFiducials =
        Arrays.stream(tagIds)
            .mapToObj(id -> new RawFiducial((int) id, 0, 0, 0, 0, 0, 0))
            .toArray(RawFiducial[]::new);
    return new PoseEstimate(
        pose,
        Utils.getCurrentTimeSeconds(),
        latencyMS,
        tagIds.length,
        0.0,
        avgTagDist,
        0.0,
        rawFiducials,
        isMegaTag2);
  }

  /** Updates the PhotonPoseEstimator and returns the estimated global pose. */
  private Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    visionEst.ifPresentOrElse(
        est ->
            getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
        });
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  private Field2d getSimDebugField() {
    if (!RobotBase.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  @FunctionalInterface
  public interface VisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
