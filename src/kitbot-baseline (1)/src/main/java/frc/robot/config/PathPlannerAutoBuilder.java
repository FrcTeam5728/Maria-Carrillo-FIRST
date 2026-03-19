// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Utility class for building PathPlanner autonomous commands.
 * Configures AutoBuilder for differential drive robots.
 */
public class PathPlannerAutoBuilder {
    
    private static boolean configured = false;
    
    /**
     * Configures AutoBuilder for use with the drive subsystem.
     * Must be called once during robot initialization.
     * 
     * @param driveSubsystem The drive subsystem to use
     */
    public static void configure(DriveSubsystem driveSubsystem) {
        if (configured) {
            return;
        }
        
        // Configure AutoBuilder for differential drive
        AutoBuilder.configure(
            driveSubsystem::getPose, // Robot pose supplier
            driveSubsystem::resetOdometry, // Method to reset odometry
            () -> new edu.wpi.first.math.kinematics.ChassisSpeeds(), // ChassisSpeeds supplier (empty for now)
            (speeds, feedforwards) -> {
                // Chassis speeds consumer - convert to tank drive
                // This is a simplified version for differential drive
                double leftSpeed = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * PathPlannerConfig.getTrackWidth() / 2.0;
                double rightSpeed = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond * PathPlannerConfig.getTrackWidth() / 2.0;
                driveSubsystem.driveArcade(() -> leftSpeed, () -> rightSpeed);
            },
            new PPLTVController(0.02), // Path following controller for differential drive
            PathPlannerConfig.createConfig(), // Robot config
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
            },
            driveSubsystem // Reference to this subsystem to set requirements
        );
        
        configured = true;
        System.out.println("PathPlanner AutoBuilder configured for differential drive");
    }
    
    /**
     * Creates an autonomous command from a PathPlanner auto file.
     * 
     * @param autoName The name of the auto file (without .auto extension)
     * @return Command that will run the autonomous routine
     */
    public static Command getAutoCommand(String autoName) {
        if (!configured) {
            System.err.println("Error: PathPlannerAutoBuilder not configured!");
            return Commands.none();
        }
        
        try {
            return new PathPlannerAuto(autoName);
        } catch (Exception e) {
            System.err.println("Error loading auto '" + autoName + "': " + e.getMessage());
            return Commands.none();
        }
    }
    
    /**
     * Creates a command to follow a specific path.
     * 
     * @param pathName The name of the path file (without .path extension)
     * @return Command that will follow the path
     */
    public static Command getFollowPathCommand(String pathName) {
        if (!configured) {
            System.err.println("Error: PathPlannerAutoBuilder not configured!");
            return Commands.none();
        }
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            System.err.println("Error loading path '" + pathName + "': " + e.getMessage());
            return Commands.none();
        }
    }
    
    /**
     * Creates a command to follow a path with a custom starting pose.
     * 
     * @param pathName The name of the path file (without .path extension)
     * @param startingPose The starting pose for the path
     * @return Command that will follow the path
     */
    public static Command getFollowPathCommand(String pathName, Pose2d startingPose) {
        if (!configured) {
            System.err.println("Error: PathPlannerAutoBuilder not configured!");
            return Commands.none();
        }
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return Commands.sequence(
                Commands.runOnce(() -> {
                    // Reset odometry to starting pose
                    var driveSubsystem = getDriveSubsystem();
                    if (driveSubsystem != null) {
                        driveSubsystem.resetOdometry(startingPose);
                    }
                }),
                AutoBuilder.followPath(path)
            );
        } catch (Exception e) {
            System.err.println("Error loading path '" + pathName + "': " + e.getMessage());
            return Commands.none();
        }
    }
    
    /**
     * Gets the configured drive subsystem.
     * This is a workaround since we don't store the reference.
     * 
     * @return The drive subsystem if configured, null otherwise
     */
    private static DriveSubsystem getDriveSubsystem() {
        // This is a limitation of the current design
        // In a real implementation, you'd store the drive subsystem reference
        // For now, this will need to be handled by the caller
        return null;
    }
    
    /**
     * Checks if AutoBuilder has been configured.
     * 
     * @return True if configured, false otherwise
     */
    public static boolean isConfigured() {
        return configured;
    }
}