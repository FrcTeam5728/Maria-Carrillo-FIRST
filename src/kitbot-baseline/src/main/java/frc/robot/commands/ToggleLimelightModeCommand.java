// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AprilTagPositionUpdater;

/**
 * Command that toggles Limelight between streaming and vision modes.
 * Pipeline 0: Vision mode for AprilTag detection
 * Pipeline 1: Streaming mode for video feed
 */
public class ToggleLimelightModeCommand extends Command {
    
    private final AprilTagPositionUpdater positionUpdater;
    
    /**
     * Creates a new ToggleLimelightModeCommand.
     * 
     * @param positionUpdater Position updater for mode tracking
     */
    public ToggleLimelightModeCommand(AprilTagPositionUpdater positionUpdater) {
        this.positionUpdater = positionUpdater;
    }
    
    @Override
    public void initialize() {
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
