// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Autonomous commands for the swerve drive robot.
 */
public final class Autos {
  /**
   * Example autonomous command that drives forward for 1 second.
   */
  public static final Command exampleAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem fuelSubsystem) {
    return new SequentialCommandGroup(
        // Drive forward at 50% speed for 1 second
        driveSubsystem.driveCommand(
            () -> 0.5,
            () -> 0.0,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Stop the robot
        driveSubsystem.stopCommand().withTimeout(0.1)
    );
  }

  /**
   * Swerve autonomous command that drives in a square pattern.
   */
  public static final Command swerveAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem fuelSubsystem) {
    return new SequentialCommandGroup(
        // Drive forward for 2 seconds
        driveSubsystem.driveCommand(
            () -> 0.5,
            () -> 0.0,
            () -> 0.0,
            false
        ).withTimeout(2.0),
        
        // Strafe right for 1 second
        driveSubsystem.driveCommand(
            () -> 0.0,
            () -> 0.5,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Drive backward for 2 seconds
        driveSubsystem.driveCommand(
            () -> -0.5,
            () -> 0.0,
            () -> 0.0,
            false
        ).withTimeout(2.0),
        
        // Strafe left for 1 second
        driveSubsystem.driveCommand(
            () -> 0.0,
            () -> -0.5,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Stop the robot
        driveSubsystem.stopCommand().withTimeout(0.1)
    );
  }

  /**
   * Autonomous command that demonstrates all swerve movements.
   */
  public static final Command swerveTestAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem fuelSubsystem) {
    return new SequentialCommandGroup(
        // Forward
        driveSubsystem.driveCommand(
            () -> 0.5,
            () -> 0.0,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Strafe right
        driveSubsystem.driveCommand(
            () -> 0.0,
            () -> 0.5,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Backward
        driveSubsystem.driveCommand(
            () -> -0.5,
            () -> 0.0,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Strafe left
        driveSubsystem.driveCommand(
            () -> 0.0,
            () -> -0.5,
            () -> 0.0,
            false
        ).withTimeout(1.0),
        
        // Rotate clockwise
        driveSubsystem.driveCommand(
            () -> 0.0,
            () -> 0.0,
            () -> 0.5,
            false
        ).withTimeout(1.0),
        
        // Stop
        driveSubsystem.stopCommand().withTimeout(0.1)
    );
  }
}
