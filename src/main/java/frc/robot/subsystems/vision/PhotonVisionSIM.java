package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.utils.FieldConstants;
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
 * Represents a simulated version of the PhotonVision system. This class extends the VisionProvider
 * class and provides methods for interacting with the simulated vision system.
 */
public class PhotonVisionSIM extends VisionProvider {
  /**
   * Constructs a PhotonVision interface with the given camera names.
   *
   * @param cameraName Name of Camera
   */
  private PhotonCamera camera;

  private PhotonPoseEstimator photonEstimator;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  private double lastEstTimestamp = 0;

  public PhotonVisionSIM(
      String[] cameraNames,
      Transform3d[] robotToCameras,
      Supplier<SwerveDriveState> swerveStateSupplier) {
    super(cameraNames, swerveStateSupplier);

    // Check if cameraNames and robotToCameras are the same length
    if (cameraNames.length != robotToCameras.length) {
      throw new IllegalArgumentException(
          "cameraNames and robotToCameras must be the same length. cameraNames: "
              + cameraNames.length
              + ", robotToCameras: "
              + robotToCameras.length);
    }

    // Create the vision system simulation which handles cameras and targets on the
    // field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(FieldConstants.aprilTags);

    for (int i = 0; i < cameraNames.length; i++) {
      // Get robotToCameras for the current camera
      String cameraName = cameraNames[i];
      var robotToCamera = robotToCameras[i];
      camera = new PhotonCamera(cameraName);
      photonEstimator =
          new PhotonPoseEstimator(
              FieldConstants.aprilTags,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              robotToCamera);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
    }
  }

  /**
   * Retrieves the vision update from the PhotonVision system.
   *
   * @return the pose estimate representing the vision update
   */
  @Override
  protected PoseEstimate[] getVisionUpdate(String cameraName) {
    visionSim.update(swerveStateSupplier.get().Pose);
    visionSim.getDebugField();
    PhotonPipelineResult results = cameraSim.getCamera().getLatestResult();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (results.targets.isEmpty() || allianceOptional.isEmpty()) {
      return new PoseEstimate[] {PoseEstimate.DEFAULT, PoseEstimate.DEFAULT};
    }
    double timestamp = results.getTimestampSeconds();
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
              poseEstimation.toPose2d(), timestamp, latencyMS, tagIDs, averageTagDistance, false);
      PoseEstimate mt2 =
          createPoseEstimate(
              poseEstimation.toPose2d(), timestamp, latencyMS, tagIDs, averageTagDistance, true);

      return new PoseEstimate[] {mt1, mt2};
    }
    return new PoseEstimate[] {PoseEstimate.DEFAULT, PoseEstimate.DEFAULT};
  }

  /** Creates a PoseEstimate object from the given parameters. */
  private PoseEstimate createPoseEstimate(
      Pose2d pose,
      double timestamp,
      double latencyMS,
      long[] tagIds,
      double avgTagDist,
      boolean isMegaTag2) {
    RawFiducial[] rawFiducials =
        Arrays.stream(tagIds)
            .mapToObj(id -> new RawFiducial((int) id, 0, 0, 0, 0, 0, 0))
            .toArray(RawFiducial[]::new);
    return new PoseEstimate(
        pose,
        timestamp,
        latencyMS / 1e3,
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
}
