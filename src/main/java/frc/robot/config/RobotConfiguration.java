package frc.robot.config;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotVersion;

public class RobotConfiguration {

  public static CompRobot compRobot = new CompRobot();
  public static ProtoRobot protoRobot = new ProtoRobot();

  public static PortConfiguration getPortConfiguration(RobotVersion version) {
    switch (version) {
      case COMPROBOT:
        return compRobot.portConfig;

      case PROTOROBOT:
        return protoRobot.portConfig;

      default:
        throw new IllegalArgumentException(
            "The current robot version is not accounted for in the switch statement!");
    }
  }

  public static CharacterizationConfiguration getCharacterizationConfiguration(
      RobotVersion version) {
    switch (version) {
      case COMPROBOT:
        return compRobot.characterizationConfig;

      case PROTOROBOT:
        return protoRobot.characterizationConfig;

      default:
        throw new IllegalArgumentException(
            "The current robot version is not accounted for in the switch statement!");
    }
  }

  public static class CompRobot {

    private static final int GYRO_ID = 20;

    private static final int FRONT_LEFT_TURN_ID = 1;
    private static final int FRONT_LEFT_DRIVE_ID = 2;
    private static final int FRONT_LEFT_ABSOLUTE_ID = 3;

    private static final int FRONT_RIGHT_TURN_ID = 4;
    private static final int FRONT_RIGHT_DRIVE_ID = 5;
    private static final int FRONT_RIGHT_ABSOLUTE_ID = 6;

    private static final int BACK_LEFT_TURN_ID = 7;
    private static final int BACK_LEFT_DRIVE_ID = 8;
    private static final int BACK_LEFT_ABSOLUTE_ID = 9;

    private static final int BACK_RIGHT_TURN_ID = 10;
    private static final int BACK_RIGHT_DRIVE_ID = 11;
    private static final int BACK_RIGHT_ABSOLUTE_ID = 12;

    private static final String LEFT_LIMELIGHT_NAME = "limelight-left";
    private static final String RIGHT_LIMELIGHT_NAME = "limelight-right";

    private static final boolean USE_PHEONIX_PRO = true;
    private static final String DRIVE_CANBUS = "ctre";

    public final PortConfiguration portConfig = new PortConfiguration();

    private static final double FRONT_LEFT_OFFSET = -0.307861;
    private static final double FRONT_RIGHT_OFFSET = -0.416748;
    private static final double BACK_LEFT_OFFSET = -0.514404;
    private static final double BACK_RIGHT_OFFSET = -0.504395;

    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.85);
    private static final double MAX_VELOCITY = Units.feetToMeters(19.5);

    public final CharacterizationConfiguration characterizationConfig =
        new CharacterizationConfiguration();

    public CompRobot() {
      portConfig.gyroID = GYRO_ID;

      portConfig.frontLeftTurnMotorID = FRONT_LEFT_TURN_ID;
      portConfig.frontLeftDriveMotorID = FRONT_LEFT_DRIVE_ID;
      portConfig.frontLeftAbsoluteEncoderID = FRONT_LEFT_ABSOLUTE_ID;

      portConfig.frontRightTurnMotorID = FRONT_RIGHT_TURN_ID;
      portConfig.frontRightDriveMotorID = FRONT_RIGHT_DRIVE_ID;
      portConfig.frontRightAbsoluteEncoderID = FRONT_RIGHT_ABSOLUTE_ID;

      portConfig.backLeftTurnMotorID = BACK_LEFT_TURN_ID;
      portConfig.backLeftDriveMotorID = BACK_LEFT_DRIVE_ID;
      portConfig.backLeftAbsoluteEncoderID = BACK_LEFT_ABSOLUTE_ID;

      portConfig.backRightTurnMotorID = BACK_RIGHT_TURN_ID;
      portConfig.backRightDriveMotorID = BACK_RIGHT_DRIVE_ID;
      portConfig.backRightAbsoluteEncoderID = BACK_RIGHT_ABSOLUTE_ID;

      portConfig.leftLimelightName = LEFT_LIMELIGHT_NAME;
      portConfig.rightLimelightName = RIGHT_LIMELIGHT_NAME;

      portConfig.usePheonixPro = USE_PHEONIX_PRO;
      portConfig.driveCANBus = DRIVE_CANBUS;

      characterizationConfig.frontLeftOffset = FRONT_LEFT_OFFSET;
      characterizationConfig.frontRightOffset = FRONT_RIGHT_OFFSET;
      characterizationConfig.backLeftOffset = BACK_LEFT_OFFSET;
      characterizationConfig.backRightOffset = BACK_RIGHT_OFFSET;

      characterizationConfig.driveGearRatio = DRIVE_GEAR_RATIO;
      characterizationConfig.wheelRadiusMeters = WHEEL_RADIUS_METERS;
      characterizationConfig.maxVelocity = MAX_VELOCITY;
    }
  }

  public static class ProtoRobot {

    private static final int GYRO_ID = 20;

    private static final int FRONT_LEFT_TURN_ID = 1;
    private static final int FRONT_LEFT_DRIVE_ID = 2;
    private static final int FRONT_LEFT_ABSOLUTE_ID = 3;

    private static final int FRONT_RIGHT_TURN_ID = 4;
    private static final int FRONT_RIGHT_DRIVE_ID = 5;
    private static final int FRONT_RIGHT_ABSOLUTE_ID = 6;

    private static final int BACK_LEFT_TURN_ID = 7;
    private static final int BACK_LEFT_DRIVE_ID = 8;
    private static final int BACK_LEFT_ABSOLUTE_ID = 9;

    private static final int BACK_RIGHT_TURN_ID = 10;
    private static final int BACK_RIGHT_DRIVE_ID = 11;
    private static final int BACK_RIGHT_ABSOLUTE_ID = 12;

    private static final String LIMELIGHT_NAME = "limelight";

    private static final boolean USE_PHEONIX_PRO = true;
    private static final String DRIVE_CANBUS = "ctre";

    public final PortConfiguration portConfig = new PortConfiguration();

    private static final double FRONT_LEFT_OFFSET = -0.626221;
    private static final double FRONT_RIGHT_OFFSET = -0.357910;
    private static final double BACK_LEFT_OFFSET = -0.424805;
    private static final double BACK_RIGHT_OFFSET = -0.589844;

    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.85);
    private static final double MAX_VELOCITY = Units.feetToMeters(15.5);

    public final CharacterizationConfiguration characterizationConfig =
        new CharacterizationConfiguration();

    public ProtoRobot() {
      portConfig.gyroID = GYRO_ID;

      portConfig.frontLeftTurnMotorID = FRONT_LEFT_TURN_ID;
      portConfig.frontLeftDriveMotorID = FRONT_LEFT_DRIVE_ID;
      portConfig.frontLeftAbsoluteEncoderID = FRONT_LEFT_ABSOLUTE_ID;

      portConfig.frontRightTurnMotorID = FRONT_RIGHT_TURN_ID;
      portConfig.frontRightDriveMotorID = FRONT_RIGHT_DRIVE_ID;
      portConfig.frontRightAbsoluteEncoderID = FRONT_RIGHT_ABSOLUTE_ID;

      portConfig.backLeftTurnMotorID = BACK_LEFT_TURN_ID;
      portConfig.backLeftDriveMotorID = BACK_LEFT_DRIVE_ID;
      portConfig.backLeftAbsoluteEncoderID = BACK_LEFT_ABSOLUTE_ID;

      portConfig.backRightTurnMotorID = BACK_RIGHT_TURN_ID;
      portConfig.backRightDriveMotorID = BACK_RIGHT_DRIVE_ID;
      portConfig.backRightAbsoluteEncoderID = BACK_RIGHT_ABSOLUTE_ID;

      portConfig.leftLimelightName = LIMELIGHT_NAME;

      portConfig.usePheonixPro = USE_PHEONIX_PRO;
      portConfig.driveCANBus = DRIVE_CANBUS;

      characterizationConfig.frontLeftOffset = FRONT_LEFT_OFFSET;
      characterizationConfig.frontRightOffset = FRONT_RIGHT_OFFSET;
      characterizationConfig.backLeftOffset = BACK_LEFT_OFFSET;
      characterizationConfig.backRightOffset = BACK_RIGHT_OFFSET;

      characterizationConfig.driveGearRatio = DRIVE_GEAR_RATIO;
      characterizationConfig.wheelRadiusMeters = WHEEL_RADIUS_METERS;
      characterizationConfig.maxVelocity = MAX_VELOCITY;
    }
  }
}
