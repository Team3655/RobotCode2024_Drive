package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO[] limelights;
  private final VisionIOInputsAutoLogged[] limeLightInputs;

  private Queue<VisionMeasurement> acceptedMeasurements = new LinkedList<>();
  private ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
  private ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

  public VisionSubsystem(VisionIO... limelights) {
    this.limelights = limelights;
    // create and fill a list of autologged inputs
    limeLightInputs = new VisionIOInputsAutoLogged[limelights.length];
    for (int i = 0; i < limelights.length; i++) {
      limeLightInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    rejectedPoses.clear();
    acceptedPoses.clear();

    for (int i = 0; i < limelights.length; i++) {

      limelights[i].updateInputs(limeLightInputs[i]);
      Logger.processInputs("Vision/" + limelights[i].getName(), limeLightInputs[i]);

      for (int j = 0; j < limeLightInputs[i].robotPose.length; j++) {

        // Basic checks to avoid errors
        if (!limeLightInputs[i].isNew
            || !limeLightInputs[i].hasValidTarget
            || limeLightInputs[i].robotPose[j] == null
            || limeLightInputs[i].targetPoses.length < 1) {
          continue;
        }

        // If the camera to target distance is too great, reject the measurement
        // depending on the number of tags in view use a different maximum
        if (limeLightInputs[i].avgDistanceToCamera
                >= (limeLightInputs[i].targetPoses.length > 1
                    ? VisionConstants.MULTI_TAG_MAXIMUM
                    : VisionConstants.SINGLE_TAG_MAXIMUM)
            || !isInField(limeLightInputs[i].robotPose[j])) {}
      }
    }
  }

  public Queue<VisionMeasurement> getMeasurements() {
    return acceptedMeasurements;
  }

  /**
   * @return Whether the robot is within the x and y coordinates of the field
   */
  private static boolean isInField(Pose2d pose) {
    if (pose.getX() >= 0
        && pose.getX() <= Units.feetToMeters(54 + (1.0 / 12.0))
        && pose.getY() >= 0
        && pose.getY() <= Units.feetToMeters(26 + (7.0 / 12.0))) {
      return true;
    }
    return false;
  }

  public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
