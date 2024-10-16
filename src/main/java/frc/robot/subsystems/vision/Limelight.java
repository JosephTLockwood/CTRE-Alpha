package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.function.Supplier;

/** Represents a Limelight vision provider. */
public class Limelight extends VisionProvider {

  /**
   * Constructs a Limelight object with the specified camera name and swerve state supplier.
   *
   * @param cameraNames the name of the camera
   * @param swerveStateSupplier the supplier for the swerve drive state
   */
  public Limelight(String[] cameraNames, Supplier<SwerveDriveState> swerveStateSupplier) {
    super(cameraNames, swerveStateSupplier);
  }

  /**
   * Retrieves the vision update from the Limelight.
   *
   * @return the pose estimate representing the vision update
   */
  @Override
  protected PoseEstimate[] getVisionUpdate(String cameraName) {
    LimelightHelpers.SetRobotOrientation(
        cameraName, swerveStateSupplier.get().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    return new PoseEstimate[] {mt1, mt2};
  }
}
