// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.utils.DynamicUSBCameraServer;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> 0),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        ballSubsystem.spinUpCommand().withTimeout(1),
        ballSubsystem.launchCommand().withTimeout(9),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
  }

  // Quick autonomous path: 300" forward, left turn, intake while driving 180", left turn, 170" forward, stop
  public static final Command quickAuto(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive forward 300 inches (25 feet) at moderate speed
        driveSubsystem.driveArcade(() -> 0.6, () -> 0).withTimeout(5.0),
        
        // Stop briefly before turn
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(0.5),
        
        // Turn left (90 degrees) - rotate for appropriate time
        driveSubsystem.driveArcade(() -> 0, () -> -0.5).withTimeout(1.5),
        
        // Stop briefly before driving with intake
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(0.3),
        
        // Drive forward 180 inches (15 feet) while engaging intake
        new ParallelCommandGroup(
            driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(3.0),
            ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()).withTimeout(3.0)
        ),
        
        // Stop briefly before second turn
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(0.5),
        
        // Turn left (90 degrees) - rotate for appropriate time
        driveSubsystem.driveArcade(() -> 0, () -> -0.5).withTimeout(1.5),
        
        // Stop briefly before final drive
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(0.3),
        
        // Drive forward 170 inches (14.17 feet)
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2.8),
        
        // Final stop
        driveSubsystem.driveArcade(() -> 0, () -> 0),
        
        // Stop intake
        ballSubsystem.runOnce(() -> ballSubsystem.stop())
    );
  }
}
