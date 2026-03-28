// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.FactoryConstants.*;

/**
 * Factory class for creating drive subsystems based on configuration.
 */
public class RobotSubsystemFactory {
  /**
   * Creates a drive subsystem based on the configured drive type.
   * 
   * @return The appropriate drive subsystem
   */
  public static DriveSubsystem createDriveSubsystem() {
    if (DRIVE_SUBSYSTEM_TYPE.equals("SWERVE")) {
      return new SwerveDriveSubsystem();
    } else if (DRIVE_SUBSYSTEM_TYPE.equals("SPARKMAX")) {
      return new DriveSubsystemSparkMax();
    } else {
      // Default to differential drive
      return new DriveSubsystemSparkMax();
    }
  }
}
