package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.VisionMode;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public class VisionIOInputs {
    public boolean hasValidTarget = false;
    public boolean isNew = false;

    public Pose3d[] targetPoses = new Pose3d[] {};
    public Pose2d[] robotPose = new Pose2d[] {};

    public double avgDistanceToCamera = 0.0;
    public double targetErrorRads = 0.0;

    public int currentPipeline = 0;

    public double captureLatencySec = 0.0;
    public double pipelineLatencySec = 0.0;
    public double jsonParseLatencySec = 0.0;
    public double timestamp = 0.0;
  }

  // Updates the set of loggable inputs
  public default void updateInputs(VisionIOInputs inputs) {}

  // Returns the name of the camera
  public default String getName() {
    return "";
  }

  public default void setMode(VisionMode mode) {}

  public default void updateRobotPose(Pose2d robotPose) {}
}
