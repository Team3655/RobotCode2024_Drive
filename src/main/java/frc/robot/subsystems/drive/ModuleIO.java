package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

/**
 * Connects the software to the hardware and directly receives data and/or sends control data to the
 * swerve drive module
 */
public interface ModuleIO {
  /**
   * Updates the swerve module values from the data receive by the module.
   *
   * @param swerveModuleIOInputs Has the current data of the swerve drive module
   */
  default void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {}

  /**
   * Sets the target steer position of the swerve drive module
   *
   * @param targetSteerPositionRad Target steer angle in radians
   */
  default void setTargetSteerPosition(double targetSteerPositionRad) {}

  /**
   * Sets the target drive velocity of the swerve drive module
   *
   * @param targetDriveVelocityMetersPerSec Target drive velocity in meters per second
   */
  default void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {}

  default void resetToAbsoluteAngle() {}

  default double getMaxVelocity() {
    return Units.feetToMeters(16.4); 
  }

  default BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[0];
  }

  /**
   * The @AutoLog notation allows AdvantageKit to automatically generate implementations of `toLog`
   * and `fromLog` for these values
   *
   * <p>These values are updated from the IO implementation (REAL or SIM) at the same timee they are
   * sent to actual hardware.
   */
  @AutoLog
  class SwerveModuleIOInputs {
    /** Drive Inputs */
    public double drivePositionRad = 0;
    /** Drive motor positioning */
    public double drivePositionMeters = 0;
    /** Drive motor speed */
    public double driveVelocityMetersPerSec = 0;
    /** Amps going to drive motor */
    public double driveCurrentDrawAmps = 0;
    /** Volts being sent to the drive motor */
    public double driveAppliedVolts = 0;

    // Steering Inputs
    /** Steering position angle */
    public double steerPositionTicks = 0;

    public double steerPositionRad = 0;

    public double steerPositionDeg = 0;
    /** Steering moto speed */
    public double steerVelocityRadPerSec = 0;
    /** Amps going to steer motor */
    public double steerCurrentDrawAmps = 0;
    /** Volts being sent to the steer motor */
    public double steerAppliedVolts = 0;

    // Steering Encoder Inputs
    /**
     * Absolute position. Angle stays the same at all times (as opposed to the motor encoders which
     * updates the angles based off of the starting angle)
     */
    public double steerAbsolutePosition = 0;

    // Drive Outputs
    public double targetDriveVelocityMetersPerSec = 0;

    // Steer Outputs
    public double targetSteerPositionRad = 0;
  }
}
