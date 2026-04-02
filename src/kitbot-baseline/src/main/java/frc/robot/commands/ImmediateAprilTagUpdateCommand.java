// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.AprilTagPositionUpdater;

/**
 * Command that immediately updates position when any AprilTag is detected.
 * This command runs continuously and updates position on every detection.
 */
public class ImmediateAprilTagUpdateCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private final AprilTagPositionUpdater positionUpdater;
    
    private int detectionCount = 0;
    private int updateCount = 0;
    private double lastDetectionTime = 0.0;
    
    /**
     * Creates a new ImmediateAprilTagUpdateCommand.
     * 
     * @param limelightSubsystem Limelight subsystem for detection
     * @param positionUpdater Position updater for calculations
     */
    public ImmediateAprilTagUpdateCommand(LimelightSubsystem limelightSubsystem, 
                                       AprilTagPositionUpdater positionUpdater) {
        this.limelightSubsystem = limelightSubsystem;
        this.positionUpdater = positionUpdater;
    }
    
    @Override
    public void initialize() {
        detectionCount = 0;
        updateCount = 0;
        lastDetectionTime = System.currentTimeMillis() / 1000.0;
    }
    
    @Override
    public void execute() {
        // Check for ANY AprilTag detection, even brief
        if (limelightSubsystem.hasTarget()) {
            int targetId = limelightSubsystem.getTargetId();
            if (targetId > 0) {
                detectionCount++;
                lastDetectionTime = System.currentTimeMillis() / 1000.0;
                
                // Attempt position update immediately
                if (positionUpdater.updatePosition()) {
                    updateCount++;
                }
            }
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
     * Gets the number of AprilTag detections.
     * 
     * @return Number of detections
     */
    public int getDetectionCount() {
        return detectionCount;
    }
    
    /**
     * Gets the number of successful position updates.
     * 
     * @return Number of updates
     */
    public int getUpdateCount() {
        return updateCount;
    }
    
    /**
     * Gets the detection rate (detections per second).
     * 
     * @return Detection rate
     */
    public double getDetectionRate() {
        double elapsed = (System.currentTimeMillis() / 1000.0) - lastDetectionTime;
        return elapsed > 0 ? detectionCount / elapsed : 0.0;
    }
    
    /**
     * Gets the success rate (updates per detection).
     * 
     * @return Success rate (0.0 to 1.0)
     */
    public double getSuccessRate() {
        return detectionCount > 0 ? (double) updateCount / detectionCount : 0.0;
    }
}
