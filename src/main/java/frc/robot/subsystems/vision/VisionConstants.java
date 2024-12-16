package frc.robot.subsystems.vision;

public class VisionConstants {

  public static final double TRANSLATION_COEFFICIENT = 0.55;
  public static final double ROTATION_COEFFICIENT = 2.0;

  // The maximum distance a measurement will be accepted in meters
  public static final double SINGLE_TAG_MAXIMUM = 4.5;
  public static final double MULTI_TAG_MAXIMUM = 7.5;

  public enum VisionMode {
    APRILTAG_LOCALIZE,
    DETECT_GAMEPIECE,
    RETRO_REFLECTIVE
  }
}
