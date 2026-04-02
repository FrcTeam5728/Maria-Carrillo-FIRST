// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AprilTagPositionUpdater;

/**
 * Command that tests AprilTag position updating.
 * Continuously updates position when AprilTags are detected.
 */
public class AprilTagPositionTestCommand extends Command {
    
    private final AprilTagPositionUpdater positionUpdater;
    
    private int updateCount = 0;
    private double lastUpdateTime = 0.0;
    
    /**
     * Creates a new AprilTagPositionTestCommand.
     * 
     * @param positionUpdater AprilTag position updater
     */
    public AprilTagPositionTestCommand(AprilTagPositionUpdater positionUpdater) {
        this.positionUpdater = positionUpdater;
    }
    
    @Override
    public void initialize() {
        updateCount = 0;
        lastUpdateTime = System.currentTimeMillis() / 1000.0;
    }
    
    @Override
    public void execute() {
        // Update position from AprilTag
        boolean updated = positionUpdater.updatePosition();
        
        if (updated) {
            updateCount++;
            lastUpdateTime = System.currentTimeMillis() / 1000.0;
        }
        
        // Update status every 2 seconds
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (currentTime - lastUpdateTime > 2.0) {
            // Status published to SmartDashboard by FieldPositionSystem
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Command cleanup
    }
    
    @Override
    public boolean isFinished() {
        // Command runs until cancelled
        return false;
    }
    
    /**
     * Gets the number of successful position updates.
     * 
     * @return Number of updates
     */
    public int getUpdateCount() {
        return updateCount;
    }
}
