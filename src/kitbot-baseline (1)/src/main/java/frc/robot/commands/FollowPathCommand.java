// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that follows a PathPlanner path using the drive subsystem.
 */
public class FollowPathCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private PathPlannerPath path;
    private final Pose2d startingPose;
    private Command followPathCommand;
    
    /**
     * Creates a new FollowPathCommand.
     * 
     * @param driveSubsystem The drive subsystem to use
     * @param pathName The name of the path to follow
     */
    public FollowPathCommand(DriveSubsystem driveSubsystem, String pathName) {
        this.driveSubsystem = driveSubsystem;
        try {
            this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.err.println("Error loading path '" + pathName + "': " + e.getMessage());
            this.path = null;
        }
        // Use a default starting pose since getStartingPose() is not available
        this.startingPose = new Pose2d();
        
        addRequirements(driveSubsystem);
    }
    
    /**
     * Creates a new FollowPathCommand with a specific starting pose.
     * 
     * @param driveSubsystem The drive subsystem to use
     * @param pathName The name of the path to follow
     * @param startingPose The starting pose for the path
     */
    public FollowPathCommand(DriveSubsystem driveSubsystem, String pathName, Pose2d startingPose) {
        this.driveSubsystem = driveSubsystem;
        try {
            this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.err.println("Error loading path '" + pathName + "': " + e.getMessage());
            this.path = null;
        }
        this.startingPose = startingPose;
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Reset odometry to starting pose
        driveSubsystem.resetOdometry(startingPose);
        
        // Create the follow path command using AutoBuilder
        followPathCommand = AutoBuilder.followPath(path);
        followPathCommand.schedule();
    }
    
    @Override
    public void execute() {
        // The actual path following is handled by the AutoBuilder command
    }
    
    @Override
    public void end(boolean interrupted) {
        if (followPathCommand != null) {
            followPathCommand.cancel();
        }
        // Stop the robot by creating a stop command
        driveSubsystem.driveArcade(() -> 0.0, () -> 0.0).schedule();
    }
    
    @Override
    public boolean isFinished() {
        return followPathCommand != null && followPathCommand.isFinished();
    }
    
    /**
     * Gets the starting pose for this path.
     * 
     * @return The starting pose
     */
    public Pose2d getStartingPose() {
        return startingPose;
    }
}
