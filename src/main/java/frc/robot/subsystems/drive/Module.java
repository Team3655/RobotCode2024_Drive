package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class Module {

  private final ModuleIO io;

  /**
   * This class is created from ModuleIO.java from the @AutoLog notation. Code needs to be built for
   * the file to be automatically constructed first. Class with the data read from the gyro
   * implementation.
   */
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  /** Module identification name (FrontLeft, FrontRight, etc.) */
  private final String name;

  /**
   * Initializes the IO and the identification name.
   *
   * @param io The IO class. Changes based off of if we're simulating or running the code on an
   *     actual machine
   * @param name The identification name of the swerve module.
   */
  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }

  /**
   * Does the math calculations necessary to get the current swerve module state to the target
   * swerve module state. Finds the fastest way to the swerve module angle and drive velocity.
   *
   * @param targetState The target swerve module state (contains the target drive velocity and
   *     target angle for the specified swerve module)
   */
  public void setTargetState(SwerveModuleState targetState) {
    double currentAngle = inputs.steerPositionRad; // Current angle of the swerve module

    double targetAngle = MathUtil.inputModulus(targetState.angle.getRadians(), 0, 2 * Math.PI);

    double modCurrentAngle = // Constrains the current angle between 0 and 2pi, i.e. "absoluteAngle"
        MathUtil.inputModulus(currentAngle, 0, 2 * Math.PI);

    double angleError = // Difference between target and current/absolute angle
        MathUtil.inputModulus(targetAngle - modCurrentAngle, -Math.PI, Math.PI);

    double resultAngle = // Resulting angle from adjusting the current angle with the angle error
        currentAngle + angleError;

    io.setTargetDriveVelocity(targetState.speedMetersPerSecond);
    io.setTargetSteerPosition(resultAngle);
  }

  // TODO: 'setTargetSteerAngle()' from the original code is never called.  Omitted from this code
  // as of right now

  /**
   * Getting the current swerve module position
   *
   * @return The drive speed and steer angle of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionMeters, new Rotation2d(inputs.steerPositionRad));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelocityMetersPerSec, new Rotation2d(inputs.steerPositionRad));
  }

  public void updateInputs() {
    if (DriverStation.isDisabled()) {
      io.resetToAbsoluteAngle();
    }

    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name + "Module", inputs);
  }

  public BaseStatusSignal[] getSignals() {
    return io.getSignals();
  }
}
