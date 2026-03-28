// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Constants for the robot.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor CAN IDs for swerve drive
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontRightDriveMotorId = 2;
    public static final int kBackLeftDriveMotorId = 3;
    public static final int kBackRightDriveMotorId = 4;
    
    // Encoder DIO ports for swerve drive
    public static final int kFrontLeftDriveEncoderId = 0;
    public static final int kFrontRightDriveEncoderId = 1;
    public static final int kBackLeftDriveEncoderId = 2;
    public static final int kBackRightDriveEncoderId = 3;
    
    // Motor constants
    public static final double kDriveMotorCurrentLimit = 40;
    public static final double kTurningMotorCurrentLimit = 30;
    public static final double kDriveRampRate = 0.1;
    public static final double kTurningRampRate = 0.1;
    
    // Physical dimensions
    public static final double kTrackWidth = 0.6; // meters
    public static final double kWheelBase = 0.5; // meters
    public static final double kModuleOffset = 0.3; // meters from center to each module
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class FactoryConstants {
    // Drive subsystem type: "SWERVE" for swerve drive, "SPARKMAX" for differential drive
    public static final String DRIVE_SUBSYSTEM_TYPE = "SWERVE";
  }
}
