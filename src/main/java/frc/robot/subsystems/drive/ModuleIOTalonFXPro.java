package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ModuleIOTalonFXPro implements ModuleIO {
  // TODO: Move module constants
  private static final int DRIVE_CURRENT_LIMIT = 35;
  private static final int TURN_CURRENT_LIMIT = 15;

  // Used to calculate feed forward for turn speed in 2nd order dynamics calculations
  public static final double TURN_kA = 0;
  public static final double TURN_kS = 0.05;
  public static final double TURN_kV = 0.1;

  // Drive PID values
  public static final double DRIVE_kP = 0.3;
  public static final double DRIVE_kV = 0.13;

  // Turn PID values
  public static final double TURN_kP = 100.0;

  /**
   * Conversion constant: From motor encoder ticks to position data (m)
   *
   * <p>rotations -> meters
   *
   * <p>Conversion constant: From motor encoder ticks to velocity data (m/s)
   */
  private double driveCoefficient;

  /**
   * From motor rotations to the module rotations (150 / 7 motor rotations : 1 full rotation of the
   * wheel [2pi])
   */
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private static final double MAX_VELOCITY = Units.feetToMeters(19.5);

  // Hardware object initialization
  /** TalonFX swerve module drive motor */
  private final TalonFX driveMotor;
  /** TalonFX swerve module turn motor */
  private final TalonFX turnMotor;
  /** Swerve module turn encoder (absolute angular position) */
  private final CANcoder turnEncoder;

  // Target variables.  Used only for data logging
  private double targetVelocityMetersPerSeconds = 0;
  private double targetTurnAngleRadians = 0;

  private final VoltageOut voltageControl = new VoltageOut(0.0);

  private final PositionVoltage turnPositionControl = new PositionVoltage(0.0);

  // Construct status signals from the Talon FX of the type 'double'
  private StatusSignal<Double> primaryDrivePositionSignal;
  private StatusSignal<Double> primaryDriveVelocitySignal;
  private StatusSignal<Double> turnPositionSignal;
  private StatusSignal<Double> turnVelocitySignal;
  private BaseStatusSignal[] signals;

  /**
   * Initializes the motors, encoders, and the settings for each of the devices
   *
   * @param driveMotorId Drive motor CAN ID
   * @param turnMotorId Turn motor CAN ID
   * @param turnEncoderId Turn encoder CAN ID
   * @param canBus The name of the CAN bus the device is connected to
   * @param absoluteOffsetRotations This is the offset applied to the angle motor's absolute encoder
   *     so that a reading of 0 degrees means the module is facing forward.
   * @param driveGearRatio The drive gear ratio of the module
   * @param maxVelocity Max velocity of the drive motor in m/s
   */
  public ModuleIOTalonFXPro(
      int driveMotorId,
      int turnMotorId,
      int turnEncoderId,
      String canBus,
      double absoluteOffsetRotations,
      double driveGearRatio,
      double maxVelocity) {

    // Circumference over drive gear ratio
    this.driveCoefficient = (2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS) / driveGearRatio;

    driveMotor = new TalonFX(driveMotorId, canBus);
    turnMotor = new TalonFX(turnMotorId, canBus);

    turnEncoder = new CANcoder(turnEncoderId, canBus);

    // #region Define drive motor configurations

    // Configure feedback source for drive motors - the internal sensor
    FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs();
    driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Current limiting the drive motors
    CurrentLimitsConfigs driveCurrentLimitConfigs = new CurrentLimitsConfigs();
    driveCurrentLimitConfigs.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT;
    driveCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

    // Basic settings/configurations for the drive motors
    MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
    driveMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    // Construct the TalonFX configuration and assign feedback and current limits
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Feedback = driveFeedbackConfigs;
    driveConfig.CurrentLimits = driveCurrentLimitConfigs;
    driveConfig.MotorOutput = driveMotorOutputConfigs;

    // PID values for drive motors
    driveConfig.Slot0.kP = DRIVE_kP;
    driveConfig.Slot0.kV = DRIVE_kV;

    // Finally, apply the configuration
    driveMotor.getConfigurator().apply(driveConfig);

    // #endregion

    // #region Define turn motor configurations

    // Configure the feedback source for the turn motors - the CANcoders
    FeedbackConfigs turnFeedbackConfigs = new FeedbackConfigs();
    turnFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnFeedbackConfigs.FeedbackRemoteSensorID = turnEncoderId;
    turnFeedbackConfigs.RotorToSensorRatio = TURN_GEAR_RATIO;

    // Current limiting the turn motors
    CurrentLimitsConfigs turnCurrentLimitsConfigs = new CurrentLimitsConfigs();
    turnCurrentLimitsConfigs.SupplyCurrentLimit = TURN_CURRENT_LIMIT;
    turnCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    // Basic settings/configurations for the turn motors
    MotorOutputConfigs turnMotorOutputConfigs = new MotorOutputConfigs();
    turnMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    turnMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    // Construct the Talon FX configuration and assign feedback and current limits
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.Feedback = turnFeedbackConfigs;
    turnConfig.CurrentLimits = turnCurrentLimitsConfigs;
    turnConfig.MotorOutput = turnMotorOutputConfigs;

    // PID values for the turn motors
    turnConfig.Slot0.kP = TURN_kP;

    // Finally, apply the coniguration
    turnMotor.getConfigurator().apply(turnConfig);

    // #endregion

    // #region Define absolute encoder configurations

    // Construct configuration objects
    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    MagnetSensorConfigs encoderMagnetSensorConfigs = new MagnetSensorConfigs();

    // Define sensor range and offsets
    encoderMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoderMagnetSensorConfigs.MagnetOffset = absoluteOffsetRotations;

    // Assign magnet sensor configs to CANcoder configs
    turnEncoderConfig.MagnetSensor = encoderMagnetSensorConfigs;

    // Assign the configurations defined above
    turnEncoder.getConfigurator().apply(turnEncoderConfig);

    // #endregion

    // #region Configure motor signals

    primaryDrivePositionSignal = driveMotor.getPosition();
    primaryDriveVelocitySignal = driveMotor.getVelocity();
    turnPositionSignal = turnEncoder.getPosition();
    turnVelocitySignal = turnEncoder.getVelocity();

    // Construct an array of Talon FX status signals
    // TODO: Why does this have to be of the type `BaseStatusSignal`?
    signals = new BaseStatusSignal[4];

    // Assign status signals to the array
    signals[0] = primaryDrivePositionSignal;
    signals[1] = primaryDriveVelocitySignal;
    signals[2] = turnPositionSignal;
    signals[3] = turnVelocitySignal;

    // #endregion
  }

  @Override
  /**
   * SwerveModuleIOInputs is a nested class within ModuleIO that contains all the data of the swerve
   * modules. This method updates the inputs going to the robot to log using Advantagekit
   */
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionMeters =
        BaseStatusSignal.getLatencyCompensatedValue(
                primaryDrivePositionSignal, primaryDriveVelocitySignal)
            * (driveCoefficient);
    inputs.driveVelocityMetersPerSec = primaryDriveVelocitySignal.getValue() * (driveCoefficient);
    inputs.driveCurrentDrawAmps = driveMotor.getSupplyCurrent().getValue();
    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValue();
    inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;
    inputs.steerPositionRad =
        Units.rotationsToRadians(
            // Physics - velocity is the slope of position
            BaseStatusSignal.getLatencyCompensatedValue(turnPositionSignal, turnVelocitySignal));
    inputs.steerVelocityRadPerSec = Units.rotationsToRadians(turnVelocitySignal.getValue());
    inputs.steerCurrentDrawAmps = turnMotor.getSupplyCurrent().getValue();
    inputs.steerAppliedVolts = turnMotor.getMotorVoltage().getValue();
    inputs.targetSteerPositionRad = targetTurnAngleRadians;
    inputs.steerAbsolutePosition =
        Units.rotationsToRadians(turnEncoder.getAbsolutePosition().getValue());
  }

  /**
   * A target state is sent from Module.java for each module. The following logic is unique to Talon
   * FX motor controlllers to drive the motor to the given position
   */
  @Override
  public void setTargetSteerPosition(double targetSteerPositionRad) {
    turnMotor.setControl(
        turnPositionControl
            .withPosition(Units.radiansToRotations(targetSteerPositionRad))
            .withFeedForward(0));
    // Update logged target steer position for Advantagekit
    this.targetTurnAngleRadians = targetSteerPositionRad;
  }

  @Override
  public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
    driveMotor.setControl(
        voltageControl.withOutput((targetDriveVelocityMetersPerSec / MAX_VELOCITY) * 12.0));
    // Update logged target drive velocity for Advantagekit
    this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
  }

  // TODO: I don't think this does anything - it is called in Module.java, though
  @Override
  public void resetToAbsoluteAngle() {}

  @Override
  public double getMaxVelocity() {
    return MAX_VELOCITY;
  }

  @Override
  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}
