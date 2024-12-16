// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.config.CharacterizationConfiguration;
import frc.robot.config.PortConfiguration;
import frc.robot.config.RobotConfiguration;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXPro;
import frc.robot.subsystems.vision.VisionConstants.VisionMode;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CommandNXT;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class RobotContainer {

  // Subsystems
  private final DriveSubsystem drive;
  private final VisionSubsystem vision;

  // Controllers
  private final CommandNXT driveJoystick = new CommandNXT(0);
  private final CommandNXT rotationJoystick = new CommandNXT(1);
  // private final GenericHID tractorController = new GenericHID(2);
  private final CommandXboxController controller = new CommandXboxController(3);

  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    // Upon bootup, the robot configures based on the CURRENTMODE: REAL, SIM, or default if none
    // is selected

    switch (Constants.currentMode) {
      case REAL:
        PortConfiguration portConfig =
            RobotConfiguration.getPortConfiguration(Constants.currentVersion);

        CharacterizationConfiguration characterizationConfiguration =
            RobotConfiguration.getCharacterizationConfiguration(Constants.currentVersion);

        // Real robot,instantiate hardware IO implementation
        vision =
            new VisionSubsystem(
                new VisionIOLimelight(portConfig.leftLimelightName)
                    .withPipeline(VisionMode.APRILTAG_LOCALIZE, 0),
                new VisionIOLimelight(portConfig.rightLimelightName)
                    .withPipeline(VisionMode.APRILTAG_LOCALIZE, 0));

        drive =
            new DriveSubsystem(
                new GyroIOPigeon2(portConfig.gyroID, 0, 0.0, portConfig.driveCANBus),
                new ModuleIOTalonFXPro(
                    portConfig.frontLeftDriveMotorID,
                    portConfig.frontLeftTurnMotorID,
                    portConfig.frontLeftAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfiguration.frontLeftOffset,
                    characterizationConfiguration.driveGearRatio,
                    characterizationConfiguration.maxVelocity),
                new ModuleIOTalonFXPro(
                    portConfig.frontRightDriveMotorID,
                    portConfig.frontRightTurnMotorID,
                    portConfig.frontRightAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfiguration.frontRightOffset,
                    characterizationConfiguration.driveGearRatio,
                    characterizationConfiguration.maxVelocity),
                new ModuleIOTalonFXPro(
                    portConfig.backLeftDriveMotorID,
                    portConfig.backLeftTurnMotorID,
                    portConfig.backLeftAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfiguration.backLeftOffset,
                    characterizationConfiguration.driveGearRatio,
                    characterizationConfiguration.maxVelocity),
                new ModuleIOTalonFXPro(
                    portConfig.backRightDriveMotorID,
                    portConfig.backRightTurnMotorID,
                    portConfig.backRightAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfiguration.backRightOffset,
                    characterizationConfiguration.driveGearRatio,
                    characterizationConfiguration.maxVelocity),
                vision);

        break;

      case SIM:
        // Simulated robot, instantiate physics SIM IO implementations

        vision = new VisionSubsystem(new VisionIO() {});

        drive =
            new DriveSubsystem(
                null,
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                vision);

        break;

      default:
        vision = new VisionSubsystem(new VisionIO() {});

        drive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);

        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveJoystick.getY() - controller.getLeftY(),
            () -> -driveJoystick.getX() - controller.getLeftX(),
            () -> -rotationJoystick.getX() - controller.getRightX()));

    driveJoystick.b1().or(controller.back()).onTrue(DriveCommands.zeroDrive(drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
