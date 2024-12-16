// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/** Add your docs here. */
public class PortConfiguration {

  // region: drivetrain
  public int gyroID;

  public int frontLeftDriveMotorID;
  public int frontLeftTurnMotorID;
  public int frontLeftAbsoluteEncoderID;

  public int frontRightDriveMotorID;
  public int frontRightTurnMotorID;
  public int frontRightAbsoluteEncoderID;

  public int backLeftDriveMotorID;
  public int backLeftTurnMotorID;
  public int backLeftAbsoluteEncoderID;

  public int backRightDriveMotorID;
  public int backRightTurnMotorID;
  public int backRightAbsoluteEncoderID;
  // endregion

  // region: shooter

  // endregion

  // region: intake

  // endregion

  // region: climber

  // endregion

  public String leftLimelightName;
  public String rightLimelightName;

  public boolean usePheonixPro;
  public String driveCANBus;
}
