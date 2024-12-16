package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {

  default void updateInputs(GyroIOInputs inputs) {}

  default BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[0];
  }

  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = new Rotation2d();
    public double pitch = 0;
    public double roll = 0;
    public double yawVelocityRadPerSec = 0;
  }
}
