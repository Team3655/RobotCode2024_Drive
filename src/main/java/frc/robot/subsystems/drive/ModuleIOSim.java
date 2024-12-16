package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
  // Wheel radius of a swerve module
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  // Simulated drive/steer motors using the accurate gearing ratios and moment of inertia
  // TODO: Where are these numbers found?
  private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getKrakenX60(1), 6.12, 0.025);
  private final DCMotorSim steerSim =
      new DCMotorSim(DCMotor.getKrakenX60(1), 150.0 / 7.0, 0.004096955);
  // PID controller for drive motor - target drive velocity to voltage calculation
  private final PIDController driveFeedback = new PIDController(10, 0, 0);
  // PID controller for steer motor - target angular position to voltage calculation
  private final PIDController steerFeedback = new PIDController(10, 0, 0);
  // Represents the swerve module encoder's absolute value
  private double steerRelativePositionRad = 0;
  // Represents a starting angular position value for our MOTOR encoder
  private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

  // Time is used to accomodate for the lack of physical encoders
  private double currentTime = 0.0;
  private double lastTime = 0.0;

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {

    lastTime = currentTime;
    currentTime = Utils.getCurrentTimeSeconds();
    double deltaTime = currentTime - lastTime;

    // Calculating target data to voltage data
    double driveAppliedVolts = driveFeedback.calculate(inputs.driveVelocityMetersPerSec);
    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
    double steerAppliedVolts = steerFeedback.calculate(inputs.steerPositionRad);
    steerAppliedVolts = MathUtil.clamp(steerAppliedVolts, -12.0, 12.0);

    // Applying calculated voltage data to simulated motors
    driveSim.setInputVoltage(driveAppliedVolts);
    steerSim.setInputVoltage(steerAppliedVolts);

    /**
     * Adding the difference in angles between the absolute and relative angles to the current angle
     * (correcting for the offset)
     */

    // The approximate number of radians the motor has turned since the last update
    double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * deltaTime;
    /**
     * Add this to the last known positions to get updated positions This keeps track of the
     * positions of simulated encoders that don't actually exist
     */
    steerRelativePositionRad += angleDiffRad;
    steerAbsolutePositionRad += angleDiffRad;

    // Wrapping absolute position from 0 to 2pi
    steerAbsolutePositionRad = MathUtil.inputModulus(steerAbsolutePositionRad, 0, 2 * Math.PI);

    // Setting the inputs within the IO class
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * WHEEL_RADIUS_METERS;
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS_METERS;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentDrawAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.steerPositionRad = steerRelativePositionRad;
    inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
    inputs.steerCurrentDrawAmps = Math.abs(steerSim.getCurrentDrawAmps());
    inputs.steerAbsolutePosition = steerAbsolutePositionRad;

    inputs.targetDriveVelocityMetersPerSec = driveFeedback.getSetpoint();
    inputs.targetSteerPositionRad = steerFeedback.getSetpoint();
  }

  @Override
  public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
    // Sets target point for PID controller to calculate voltage
    driveFeedback.setSetpoint(targetDriveVelocityMetersPerSec);
  }

  @Override
  public void setTargetSteerPosition(double targetSteerPositionRad) {
    // Sets targt point for PID controller to calculate voltage
    steerFeedback.setSetpoint(targetSteerPositionRad);
  }
}
