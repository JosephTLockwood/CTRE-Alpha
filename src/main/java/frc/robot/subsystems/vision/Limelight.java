package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.function.Supplier;

public class Limelight extends VisionProvider {

  public Limelight(String cameraName, Supplier<SwerveDriveState> swerveStateSupplier) {
    super(cameraName, swerveStateSupplier);
  }

  @Override
  protected PoseEstimate getVisionUpdate() {
    LimelightHelpers.SetRobotOrientation(
        cameraName, swerveStateSupplier.get().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    writePoseEstimate("Odometry/MT1/" + cameraName, mt1);
    writePoseEstimate("Odometry/MT2/" + cameraName, mt2);
    return filterPoseEstimate(mt1, mt2, swerveStateSupplier);
  }
}
