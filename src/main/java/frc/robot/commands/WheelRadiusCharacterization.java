// This file is based on code from team 6328 Mechanical Advantage
// See here for the original source:
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/c0c6d11547769f6dc5f304d5c18c9b51086a691b/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterization extends Command {
  private final DoubleSupplier gyroYawRadsSupplier;

  private final CommandSwerveDrivetrain drive;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  private SwerveRequest.RobotCentric req =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withVelocityX(0)
          .withVelocityY(0);

  public WheelRadiusCharacterization(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    this.gyroYawRadsSupplier = () -> drive.getState().Pose.getRotation().getRadians();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.setControl(req.withRotationalRate(omegaLimiter.calculate(1.0)));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = 0.0;
    if (averageWheelPosition != 0.0) {
      currentEffectiveWheelRadius =
          (accumGyroYawRads * TunerConstants.kRobotRadius.baseUnitMagnitude())
              / averageWheelPosition;
    }

    SmartDashboard.putNumber("Test Wheel Positon", averageWheelPosition);
    SmartDashboard.putNumber("Test Yaw Delta", accumGyroYawRads);
    SmartDashboard.putNumber(
        "Est. Wheel Radius", Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
