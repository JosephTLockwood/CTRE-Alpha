package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
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
import frc.robot.generated.TunerConstants;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.SignalHandler;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
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
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
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
    if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
      return;
    }
    double xyStdDev = calculateXYStdDev(mt);
    double thetaStdDev = mt.isMegaTag2 ? 9999999 : calculateThetaStdDev(mt);
    SignalHandler.getOrWriteSignal(
        "Odometry/" + cameraName,
        new double[] {mt.pose.getX(), mt.pose.getY(), mt.pose.getRotation().getDegrees()});
    poseEstimates.add(new Pair<>(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
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

  private PoseEstimate getVisionUpdate(String cameraName) {
    visionSim.update(swerveStateSupplier.get().Pose);
    visionSim.getDebugField();
    PhotonPipelineResult results = cameraSim.getCamera().getLatestResult();
    double timestamp = results.getTimestampSeconds();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (results.targets.isEmpty() || allianceOptional.isEmpty()) {
      return new PoseEstimate();
    }
    double latencyMS = results.getLatencyMillis();
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
    if (estimatedPose.isPresent()) {
      Pose3d poseEstimation = estimatedPose.get().estimatedPose;
      double averageTagDistance = 0.0;
      int[] tagIDs = new int[results.targets.size()];
      for (int i = 0; i < results.targets.size(); i++) {
        tagIDs[i] = results.targets.get(i).getFiducialId();
        var tagPose = photonEstimator.getFieldTags().getTagPose(tagIDs[i]);
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
          makePoseEstimate(
              poseEstimation.toPose2d(), timestamp, latencyMS, tagIDs, averageTagDistance, false);
      PoseEstimate mt2 =
          makePoseEstimate(
              poseEstimation.toPose2d(), timestamp, latencyMS, tagIDs, averageTagDistance, true);

      PoseEstimate mt = VisionHelper.filterPoseEstimate(mt1, mt2, swerveStateSupplier);
      if (Boolean.FALSE.equals(LimelightHelpers.validPoseEstimate(mt))) {
        return new PoseEstimate();
      }
      VisionHelper.writePoseEstimate("Odometry/MT1/" + cameraName, mt1);
      VisionHelper.writePoseEstimate("Odometry/MT2/" + cameraName, mt2);

      return mt;
    }
    return new PoseEstimate();
  }

  private PoseEstimate makePoseEstimate(
      Pose2d pose,
      double timestampSeconds,
      double latencyMS,
      int[] tagIds,
      double avgTagDist,
      boolean isMegaTag2) {
    PoseEstimate poseEstimate = new PoseEstimate();
    poseEstimate.pose = pose;
    poseEstimate.timestampSeconds = timestampSeconds;
    poseEstimate.latency = latencyMS / 1e3;
    poseEstimate.tagCount = tagIds.length;
    poseEstimate.avgTagDist = avgTagDist;
    poseEstimate.isMegaTag2 = isMegaTag2;
    poseEstimate.rawFiducials =
        Arrays.stream(tagIds)
            .mapToObj(id -> new RawFiducial(id, 0, 0, 0, 0, 0, 0))
            .toArray(RawFiducial[]::new);
    return poseEstimate;
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

  private SignalData<int[]> getOrWriteFiducials(String signalPath, int[] fiducials) {
    return SignalHandler.getOrWriteSignal(signalPath, fiducials);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  private Field2d getSimDebugField() {
    if (!RobotBase.isSimulation()) return null;
    return visionSim.getDebugField();
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
