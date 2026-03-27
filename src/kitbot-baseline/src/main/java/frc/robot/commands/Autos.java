// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
        // Repeat pattern: shoot for 2 seconds, intake for 1 second, for 15 seconds total
        // This creates 5 complete cycles (5 * 3 = 15 seconds)
        new SequentialCommandGroup(
            ballSubsystem.launchCommand().withTimeout(2),
            ballSubsystem.intakeCommand().withTimeout(1)
        ).repeatedly().withTimeout(15),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
  }

  /**
   * Swerve drive autonomous command that demonstrates full swerve capabilities.
   * Uses field-relative driving and strafing movements.
   */
  public static final Command swerveAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Strafe right for 1 second (demonstrates swerve capability)
        driveSubsystem.driveCommand(
            () -> 0.0,  // No forward movement
            () -> 1.0,  // Full strafe right
            () -> 0.0,  // No rotation
            true        // Field relative
        ).withTimeout(1.0),
        
        // Drive forward while rotating for 2 seconds (complex swerve movement)
        driveSubsystem.driveCommand(
            () -> 1.0,  // Full forward
            () -> 0.0,  // No strafe
            () -> 0.5,  // Half rotation
            true        // Field relative
        ).withTimeout(2.0),
        
        // Stop driving
        driveSubsystem.stopCommand(),
        
        // Repeat pattern: shoot for 2 seconds, intake for 1 second, for 15 seconds total
        // This creates 5 complete cycles (5 * 3 = 15 seconds)
        new SequentialCommandGroup(
            ballSubsystem.launchCommand().withTimeout(2),
            ballSubsystem.intakeCommand().withTimeout(1)
        ).repeatedly().withTimeout(15),
        
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()),
        
        // Final swerve movement - drive backward while strafing left
        driveSubsystem.driveCommand(
            () -> -0.5,  // Backward
            () -> -0.5,  // Strafe left
            () -> 0.0,   // No rotation
            false        // Robot relative
        ).withTimeout(1.5),
        
        // Final stop
        driveSubsystem.stopCommand()
    );
  }

  /**
   * Quick swerve auto for testing - simple forward movement with shooting.
   */
  public static final Command quickAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive forward for 2 seconds
        driveSubsystem.driveCommand(
            () -> 1.0,  // Full forward
            () -> 0.0,  // No strafe
            () -> 0.0,  // No rotation
            true        // Field relative
        ).withTimeout(2.0),
        
        // Stop and shoot for 3 seconds
        driveSubsystem.stopCommand().alongWith(
            ballSubsystem.launchCommand().withTimeout(3)
        ),
        
        // Stop launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop())
    );
  }

  /**
   * PathPlanner autonomous that follows the "example" path.
   * 
   * @param driveSubsystem The swerve drive subsystem
   * @param ballSubsystem The fuel subsystem
   * @return Command that follows the path and shoots
   */
  public static final Command pathPlannerExampleAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Follow the example path
        FollowPathPlannerPath.followNamedPath("example", driveSubsystem, true),
        
        // Stop and shoot for 3 seconds
        driveSubsystem.stopCommand().alongWith(
            ballSubsystem.launchCommand().withTimeout(3)
        ),
        
        // Stop launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop())
    );
  }

  /**
   * Creates a PathPlanner autonomous command from a named auto.
   * 
   * @param autoName The name of the auto from autos.json
   * @param driveSubsystem The swerve drive subsystem  
   * @param ballSubsystem The fuel subsystem
   * @return Command that runs the full PathPlanner autonomous
   */
  public static final Command pathPlannerAuto(String autoName, SwerveDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Run the PathPlanner auto
        FollowPathPlannerPath.createAutoCommand(autoName),
        
        // Ensure everything is stopped at the end
        driveSubsystem.stopCommand().alongWith(
            ballSubsystem.runOnce(() -> ballSubsystem.stop())
        )
    );
  }

  /**
   * Custom PathPlanner auto with path following and shooting sequence.
   * 
   * @param driveSubsystem The swerve drive subsystem
   * @param ballSubsystem The fuel subsystem
   * @return Command with path following and shooting
   */
  public static final Command customPathPlannerAuto(SwerveDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Follow the example path
        FollowPathPlannerPath.followNamedPath("example", driveSubsystem, true),
        
        // Repeat pattern: shoot for 2 seconds, intake for 1 second, for 15 seconds total
        new SequentialCommandGroup(
            ballSubsystem.launchCommand().withTimeout(2),
            ballSubsystem.intakeCommand().withTimeout(1)
        ).repeatedly().withTimeout(15),
        
        // Stop everything
        driveSubsystem.stopCommand().alongWith(
            ballSubsystem.runOnce(() -> ballSubsystem.stop())
        )
    );
  }
}
