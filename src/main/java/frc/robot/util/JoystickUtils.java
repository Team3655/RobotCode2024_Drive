package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class JoystickUtils {

  /**
   * Squares the joystick input and uses clever math to compensate for the offset caused by the
   * deadband
   *
   * @param input The input from the joystick
   * @param deadband The deadband value for the input
   * @return The corrected joystick values
   */
  public static double curveInput(double input, double deadband) {

    if (MathUtil.applyDeadband(input, deadband) == 0) return 0;

    // Produces linear output with a deadband zone
    double linearDeadbandValue = (input - (deadband * Math.signum(input))) / (1 - deadband);

    // Raises input to a specified power for a smoother feel
    double curvedDeadbandValue = Math.copySign(Math.pow(Math.abs(linearDeadbandValue), 2), input);

    return curvedDeadbandValue;
  }

  public static Translation2d curveTranslation2d(Translation2d translation, double deadband) {
    // Gets the length and angle of the Translation2d (vector)
    double norm = translation.getNorm();
    Rotation2d angle = translation.getAngle();

    // Applies outer deadband (combinatino of x and y motion can create vector larger than 1)
    if (norm > 1) norm = 1;

    // Curves the length of the vector for smoother feel
    double curvedNorm = curveInput(norm, deadband);

    // Create new curved Translation2d
    translation = new Translation2d(curvedNorm, angle);

    return translation;
  }
}
