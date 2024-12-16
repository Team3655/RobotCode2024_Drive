package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.JoystickUtils;
import java.util.function.DoubleSupplier;

public class DriveCommands {

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities)
   */
  public static Command joystickDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        // action to run
        () -> {
          // Construct Translation2d from joystick inputs
          Translation2d linearVelocity = getDriveTranslation(xSupplier, ySupplier);

          // the rotational rate of the robot (half by default - "rotating too fast sucks")
          double omega =
              JoystickUtils.curveInput(
                      omegaSupplier.getAsDouble(), DriveConstants.DRIVE_JOYSTICK_DEADBAND)
                  * DriveConstants.MAX_ROTATIONAL_PERCENTAGE;

          // Convert to field relative speeds and send command
          drive.setTargetVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxVelocityMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxVelocityMetersPerSec(),
                  omega * drive.getMaxAngularVelocityRadPerSec(),
                  drive.getPose().getRotation()));
        },
        drive);
  }

  public static Command zeroDrive(DriveSubsystem drive) {
    return Commands.runOnce(
            () -> drive.resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())))
        .ignoringDisable(true);
  }

  /**
   * Creates a new Translation2d for driving based off of an x and y output percent. This
   * translation has been curved to make the stick feel smoother and offer more control at lower
   * speeds
   *
   * @param xSupplier The x-input of the joystick
   * @param ySupplier The y-input of the joystick
   * @return A new translation ready to be used for driving
   */
  public static Translation2d getDriveTranslation(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    // Apply deadband, and curve joystick inputs

    double linearMagnitude =
        JoystickUtils.curveInput(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
            DriveConstants.DRIVE_JOYSTICK_DEADBAND);

    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      linearDirection = linearDirection.rotateBy(Rotation2d.fromDegrees(180));
    }

    // Calculate new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
