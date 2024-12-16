package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {

  /**
   * Pigeon2 gyroscope objects. Receives direct data from the gyroscope about the yaw, pitch, and
   * roll
   */
  private final Pigeon2 gyro;

  private StatusSignal<Double> yawSignal;
  private StatusSignal<Double> pitchSignal;
  private StatusSignal<Double> rollSignal;
  private StatusSignal<Double> angularVelocitySignal;

  private BaseStatusSignal[] signals;

  /**
   * Pigeon2 gyroIO implementation
   *
   * @param canId is the unique identifier to the gyroscope used on the robot
   * @param canBus the name of the CAN bus the device is connected to
   */
  public GyroIOPigeon2(int canId, int mountPose, double error, String canBus) {
    gyro = new Pigeon2(canId, canBus);

    Pigeon2Configuration config = new Pigeon2Configuration();
    MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
    mountPoseConfigs.MountPoseYaw = mountPose;
    GyroTrimConfigs gyroTrimConfigs = new GyroTrimConfigs();
    gyroTrimConfigs.GyroScalarZ = error;
    config.MountPose = mountPoseConfigs;
    config.GyroTrim = gyroTrimConfigs;
    gyro.getConfigurator().apply(config);

    yawSignal = gyro.getYaw();
    pitchSignal = gyro.getPitch();
    rollSignal = gyro.getRoll();
    angularVelocitySignal = gyro.getAngularVelocityZDevice();
    signals = new BaseStatusSignal[4];
    signals[0] = yawSignal;
    signals[1] = angularVelocitySignal;
    signals[2] = pitchSignal;
    signals[3] = rollSignal;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(signals).isOK();
    inputs.yaw =
        Rotation2d.fromDegrees(
            BaseStatusSignal.getLatencyCompensatedValue(yawSignal, angularVelocitySignal));
    inputs.pitch = Units.degreesToRadians(pitchSignal.getValue());
    inputs.roll = Units.degreesToRadians(rollSignal.getValue());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(angularVelocitySignal.getValue());
  }
}
