package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  private final double maxVelocityMetersPerSec;

  private final double driveBaseRadius;

  private final double maxAngularVelocityRadPerSec;

  private final GyroIO gyroIO;

  private final VisionSubsystem vision;

  /**
   * This class is created from GyroIO.java from the @AutoLog notation. Code needs to be built for
   * the file to be automatically constructed first. Class with the data read from the gyro
   * implementation.
   */
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // SwerveModuleState includes velocity of the modules
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  // SwerveModulePosition includes the distance traveled by the modules
  private final SwerveModulePosition[] swerveModulePositions;

  private final Module[] swerveModules;

  @Getter private final SwerveDriveKinematics kinematics;

  /**
   * Allows us to track the robot's position using the swerve module positions on the robot and the
   * current values of each module
   */
  @Getter private final SwerveDriveOdometry odometry;

  /** Same as odometry but with vision measurements */
  @Getter private final SwerveDrivePoseEstimator estimator;

  private final OdometryUpdateThread odometryUpdateThread;

  /**
   * The target speed of the entire robot (not just individual modules). We use this object to
   * calculate the target speeds for each individual module
   */
  private ChassisSpeeds targetVelocity = new ChassisSpeeds();

  private boolean shouldUseVisionData = true;

  /**
   * TODO: Check out 6328:
   * https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/RobotState.java
   * instead of the following OdometryUpdateThread
   */
  private class OdometryUpdateThread extends Thread {
    private BaseStatusSignal[] allSignals;
    public int successfulDataAcquisitions = 0;
    public int failedDataAcquisitions = 0;

    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;
    private double averageLoopTime = 0;

    public OdometryUpdateThread() {
      ArrayList<BaseStatusSignal> signalsList = new ArrayList<>();
      // Visual queue - 4 status signals * 4 swerve modules + 2 from...?
      // TODO: allsignals....wat
      allSignals = new BaseStatusSignal[(4 * 4) + 2];

      // Adds 4 signals from each swerve module to signalsList
      for (int i = 0; i < 4; i++) {
        signalsList.addAll(Arrays.asList(swerveModules[i].getSignals()));
      }
      // Adds a blank BaseStatusSignal array to the list
      signalsList.addAll(Arrays.asList(gyroIO.getSignals()));
      // Adds everything to the array - seems to only be the swerve modules
      allSignals = signalsList.toArray(new BaseStatusSignal[0]);
    }

    // TODO: This entire "run" method is a mystery and needs to be dissected
    public void run() {
      for (var signal : allSignals) {
        if (signal instanceof StatusSignal) {
          ((StatusSignal<?>) signal).setUpdateFrequency(250);
        }
      }
      while (true) {
        var status = BaseStatusSignal.waitForAll(0.1, allSignals);
        lastTime = currentTime;
        currentTime = Utils.getCurrentTimeSeconds();
        // calculate loop time and run it through a lowpass to make it more readable
        averageLoopTime = lowpass.calculate(currentTime - lastTime);
        if (status.isOK()) {
          successfulDataAcquisitions++;
        } else {
          failedDataAcquisitions++;
          continue;
        }

        synchronized (swerveModules) {
          synchronized (swerveModulePositions) {
            /* Now update odometry */
            for (int i = 0; i < 4; ++i) {
              swerveModules[i].updateInputs();
              swerveModulePositions[i] = swerveModules[i].getPosition();
            }
          }
        }
        // Assume Pideon2 is flat-and-level so latency compensation can be performed

        synchronized (gyroIO) {
          synchronized (gyroInputs) {
            gyroIO.updateInputs(gyroInputs);
          }
        }
        synchronized (odometry) {
          synchronized (swerveModulePositions) {
            synchronized (gyroInputs) {
              odometry.update(gyroInputs.yaw, swerveModulePositions);
            }
          }
        }
        synchronized (estimator) {
          synchronized (swerveModulePositions) {
            synchronized (gyroInputs) {
              estimator.update(gyroInputs.yaw, swerveModulePositions);
            }
          }
        }
      }
    }

    public double getAverageLoopTime() {
      return averageLoopTime;
    }

    public int getSuccessfulDataAcquisitions() {
      return successfulDataAcquisitions;
    }

    public int getFailedDataAcquisitions() {
      return failedDataAcquisitions;
    }
  }

  public DriveSubsystem(
      GyroIO gyroIO,
      ModuleIO frontLeftSwerveModuleIO,
      ModuleIO frontRightSwerveModuleIO,
      ModuleIO backLeftSwerveModuleIO,
      ModuleIO backRightSwerveModuleIO,
      VisionSubsystem vision) {

    maxVelocityMetersPerSec = frontLeftSwerveModuleIO.getMaxVelocity();
    driveBaseRadius =
        Math.hypot(DriveConstants.WHEELBASE_METERS / 2, DriveConstants.TRACK_METERS / 2);
    maxAngularVelocityRadPerSec = maxVelocityMetersPerSec / driveBaseRadius;

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::setTargetVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0), // Translation PID constants
            new PIDConstants(5, 0, 0), // Rotation PID constants
            maxVelocityMetersPerSec,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> {
          // Boolean to determine if path will be mirrored for red alliance
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    if (gyroIO == null) gyroIO = new GyroIOSim(this);
    this.gyroIO = gyroIO;
    this.vision = vision;

    swerveModules =
        new Module[] {
          new Module(frontLeftSwerveModuleIO, "FrontLeft"),
          new Module(frontRightSwerveModuleIO, "FrontRight"),
          new Module(backLeftSwerveModuleIO, "BackLeft"),
          new Module(backRightSwerveModuleIO, "BackRight")
        };

    swerveModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    var frontLeftLocation =
        new Translation2d(DriveConstants.WHEELBASE_METERS / 2, DriveConstants.TRACK_METERS / 2);
    var frontRightLocation =
        new Translation2d(DriveConstants.WHEELBASE_METERS / 2, -DriveConstants.TRACK_METERS / 2);
    var backLeftLocation =
        new Translation2d(-DriveConstants.WHEELBASE_METERS / 2, DriveConstants.TRACK_METERS / 2);
    var backRightLocation =
        new Translation2d(-DriveConstants.WHEELBASE_METERS / 2, -DriveConstants.TRACK_METERS / 2);

    kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    odometry =
        new SwerveDriveOdometry(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d());

    estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d(),
            // State standard deviations
            // Increase to trust state less
            VecBuilder.fill(0.1, 0.1, 0.1),
            // Vision pose standard deviations
            // Increase to trust vision
            // Not used, only a fallback for if the estimation coefficent fails
            VecBuilder.fill(0.5, 0.5, 0.5));

    odometryUpdateThread = new OdometryUpdateThread();
    odometryUpdateThread.start();
  }

  @Override
  public void periodic() {
    // Logging the gyro readings.  Goes to AdvantageScope
    Logger.processInputs("Drive/Gryo", gyroInputs);

    SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];

    // Using the target chassis speed (speed of the entire robot), we calculate
    // the angle and speed for each swerve drive module by creating separate
    // swerve drive module states.

    swerveModuleStates = kinematics.toSwerveModuleStates(targetVelocity);

    // Making sure that one module isn't going faster than it's
    // allowed max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocityMetersPerSec);

    // Making sure that the swerve module wheel can get to the desired state
    // as quickly and efficiently as possible
    // Linking this with the PIDController' continuous input means that the
    // motor will never turn a wheel more than 90 degrees
    synchronized (swerveModules) {
      for (int i = 0; i < optimizedSwerveModuleStates.length; i++) {
        optimizedSwerveModuleStates[i] =
            SwerveModuleState.optimize(swerveModuleStates[i], swerveModules[i].getPosition().angle);
        swerveModules[i].setTargetState(optimizedSwerveModuleStates[i]);
      }
    }

    Logger.recordOutput("Drive/Angle", getPose().getRotation());
    Logger.recordOutput("Drive/ModuleStates", swerveModuleStates);
    Logger.recordOutput(
        "Drive/TargetChassisVelocity",
        new double[] {
          targetVelocity.vxMetersPerSecond,
          targetVelocity.vyMetersPerSecond,
          targetVelocity.omegaRadiansPerSecond
        });
    Logger.recordOutput("Drive/ModuleStates", swerveModuleStates); // Logging each module state
    Logger.recordOutput(
        "Drive/OptimizedModuleStates",
        optimizedSwerveModuleStates); // Logging each optimized module state

    // Figures out the current location and rotation of the robot on the field using vision data
    Logger.recordOutput("Drive/UsingVision?", shouldUseVisionData);

    if (shouldUseVisionData) {
      synchronized (estimator) {
        for (int i = 0; i < vision.getMeasurements().size(); i++) {
          // poll the queue of measurements, this will get and remove the oldest measurement
          var measurement = vision.getMeasurements().poll();
          estimator.addVisionMeasurement(
              measurement.pose(), measurement.timestamp(), measurement.stdDevs());
        }
      }
    }

    synchronized (estimator) {
      Logger.recordOutput("Drive/EstimatedPose", estimator.getEstimatedPosition());
    }

    synchronized (odometry) {
      Logger.recordOutput("Drive/OdometryPose", estimator.getEstimatedPosition());
    }

    Logger.recordOutput(
        "Drive/OdometryThread/Average Loop Time", odometryUpdateThread.getAverageLoopTime());

    Logger.recordOutput(
        "Drive/OdometryThread/Successful Data Acquisitions",
        odometryUpdateThread.getSuccessfulDataAcquisitions());

    Logger.recordOutput(
        "Drive/OdometryThread/Failed Data Acquisitions",
        odometryUpdateThread.getFailedDataAcquisitions());
  }

  /**
   * Sets the desired drivetrain speed. The drivetrain will attempt to achieve this speed
   *
   * @param targetVelocity The target ChassisSpeeds object
   */
  public void setTargetVelocity(ChassisSpeeds targetVelocity) {
    this.targetVelocity = targetVelocity;
  }

  public Pose2d getOdomPose() {
    synchronized (odometry) {
      return odometry.getPoseMeters();
    }
  }

  public Pose2d getPose() {
    return getPose(false);
  }

  public Pose2d getPose(boolean visionAngle) {
    synchronized (estimator) {
      synchronized (odometry) {
        return new Pose2d(
            estimator.getEstimatedPosition().getX(),
            estimator.getEstimatedPosition().getY(),
            visionAngle
                ? estimator.getEstimatedPosition().getRotation()
                : odometry.getPoseMeters().getRotation());
      }
    }
  }

  public Module[] getSwerveModules() {
    synchronized (swerveModules) {
      return swerveModules;
    }
  }

  public void resetPose() {
    Pose2d pose = getPose();
    resetPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
  }

  public void resetPose(Pose2d poseMeters) {
    synchronized (odometry) {
      synchronized (swerveModulePositions) {
        synchronized (gyroInputs) {
          odometry.resetPosition(gyroInputs.yaw, swerveModulePositions, poseMeters);
        }
      }
    }

    synchronized (estimator) {
      synchronized (swerveModulePositions) {
        synchronized (gyroInputs) {
          estimator.resetPosition(gyroInputs.yaw, swerveModulePositions, poseMeters);
        }
      }
    }
  }

  public GyroIOInputsAutoLogged getGyroInputs() {
    synchronized (gyroInputs) {
      return gyroInputs;
    }
  }

  public Pose2d getProjectedPose(double latency, boolean visionAngle) {
    ChassisSpeeds currentVelocity = getChassisSpeeds();

    double xSinceLastPose = currentVelocity.vxMetersPerSecond * latency;
    double ySinceLastPose = currentVelocity.vyMetersPerSecond * latency;
    double angleSinceLastPose = currentVelocity.omegaRadiansPerSecond * latency;

    Twist2d poseDelta = new Twist2d(xSinceLastPose, ySinceLastPose, angleSinceLastPose);
    return getPose(visionAngle).exp(poseDelta);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(swerveModuleStates);
  }

  public double getMaxVelocityMetersPerSec() {
    return maxVelocityMetersPerSec;
  }

  public double getMaxAngularVelocityRadPerSec() {
    return maxAngularVelocityRadPerSec;
  }

  public void setShouldUseVisionData(boolean shouldUseVisionData) {
    this.shouldUseVisionData = shouldUseVisionData;
  }

  public double getAngularVelocity() {
    synchronized (gyroInputs) {
      return gyroInputs.yawVelocityRadPerSec;
    }
  }
}
