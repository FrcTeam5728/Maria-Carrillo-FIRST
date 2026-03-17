// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.AprilTagPositionUpdater;

/**
 * Command that toggles Limelight between streaming and vision modes.
 * Pipeline 0: Vision mode for AprilTag detection
 * Pipeline 1: Streaming mode for video feed
 */
public class ToggleLimelightModeCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private final AprilTagPositionUpdater positionUpdater;
    
    private boolean originalMode;
    
    /**
     * Creates a new ToggleLimelightModeCommand.
     * 
     * @param limelightSubsystem Limelight subsystem
     * @param positionUpdater Position updater for mode tracking
     */
    public ToggleLimelightModeCommand(LimelightSubsystem limelightSubsystem, 
                                    AprilTagPositionUpdater positionUpdater) {
        this.limelightSubsystem = limelightSubsystem;
        this.positionUpdater = positionUpdater;
    }
    
    @Override
    public void initialize() {
        // Store original mode for potential restoration
        originalMode = positionUpdater.isStreaming();
        
        // Toggle the mode
        positionUpdater.toggleMode();
    }
    
    @Override
    public void execute() {
        // Command completes immediately in initialize()
    }
    
    @Override
    public void end(boolean interrupted) {
        // Mode already toggled, no cleanup needed
    }
    
    @Override
    public boolean isFinished() {
        return true; // Command completes immediately
    }
    
    /**
     * Gets the current mode description.
     * 
     * @return Mode description string
     */
    public String getCurrentMode() {
        return positionUpdater.isStreaming() ? "STREAMING (Pipeline 1)" : "VISION (Pipeline 0)";
    }
}
