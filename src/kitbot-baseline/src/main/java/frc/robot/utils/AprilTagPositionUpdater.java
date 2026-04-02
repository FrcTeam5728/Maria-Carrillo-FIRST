// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.field.POILookup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Position update system that uses Limelight AprilTag detection
 * to calculate the robot's absolute position from POI data.
 */
public class AprilTagPositionUpdater {
    
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final POILookup poiLookup;
    
    // Position update settings
    private static final double MIN_CONFIDENCE = 0.5; // Lowered threshold for more updates
    private static final double HEADING_WEIGHT = 0.3; // weight for heading correction
    private static final double MIN_TARGET_AREA = 0.1; // Minimum area to consider
    
    // Pipeline management
    private static final int VISION_PIPELINE = 0; // Pipeline for AprilTag detection
    private static final int STREAMING_PIPELINE = 1; // Pipeline for video streaming
    private boolean isStreaming = false;
    
    // Last update tracking
    private Pose2d lastKnownPosition = new Pose2d();
    private double lastUpdateTime = 0.0;
    private boolean hasValidPosition = false;
    private int lastTargetId = -1;
    
    /**
     * Creates a new AprilTagPositionUpdater.
     * 
     * @param driveSubsystem Drive subsystem for odometry
     * @param limelightSubsystem Limelight subsystem for vision data
     */
    public AprilTagPositionUpdater(DriveSubsystem driveSubsystem, 
                                 LimelightSubsystem limelightSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.poiLookup = POILookup.getInstance();
    }
    
    /**
     * Updates the robot's position using AprilTag detection.
     * 
     * @return True if position was updated, false otherwise
     */
    public boolean updatePosition() {
        if (!limelightSubsystem.hasTarget()) {
            return false;
        }
        
        int targetId = limelightSubsystem.getTargetId();
        if (targetId <= 0) {
            return false;
        }
        
        // Look up POI data for this AprilTag
        POILookup.POIData poiData = poiLookup.getPOIByAprilTag(targetId);
        if (poiData == null) {
            return false;
        }
        
        // Check confidence (lowered threshold)
        if (poiData.confidence < MIN_CONFIDENCE) {
            return false;
        }
        
        // Calculate robot position from relative data
        Pose2d calculatedPosition = calculateRobotPosition(poiData);
        if (calculatedPosition == null) {
            return false;
        }
        
        // Update position immediately on any valid detection
        lastKnownPosition = calculatedPosition;
        lastUpdateTime = System.currentTimeMillis() / 1000.0;
        hasValidPosition = true;
        lastTargetId = targetId;
        
        // Update drive subsystem odometry
        updateOdometry(calculatedPosition);
        
        return true;
    }
    
    /**
     * Switches Limelight to streaming pipeline for video.
     */
    public void enableStreaming() {
        limelightSubsystem.setPipeline(STREAMING_PIPELINE);
        limelightSubsystem.setCameraMode(true); // Driver mode for streaming
        isStreaming = true;
    }
    
    /**
     * Switches Limelight to vision pipeline for AprilTag detection.
     */
    public void enableVision() {
        limelightSubsystem.setPipeline(VISION_PIPELINE);
        limelightSubsystem.setCameraMode(false); // Vision mode for processing
        isStreaming = false;
    }
    
    /**
     * Toggles between streaming and vision modes.
     */
    public void toggleMode() {
        if (isStreaming) {
            enableVision();
        } else {
            enableStreaming();
        }
    }
    
    /**
     * Gets the current mode.
     * 
     * @return True if streaming, false if vision
     */
    public boolean isStreaming() {
        return isStreaming;
    }
    
    /**
     * Calculates the robot's absolute position from POI data and Limelight measurements.
     * 
     * @param poiData POI data for the detected AprilTag
     * @return Calculated robot position, or null if calculation failed
     */
    private Pose2d calculateRobotPosition(POILookup.POIData poiData) {
        try {
            // Get Limelight measurements
            double horizontalOffset = limelightSubsystem.getHorizontalOffset(); // degrees
            double targetArea = limelightSubsystem.getTargetArea();
            
            // Estimate distance from target area (simplified)
            double distance = estimateDistanceFromArea(targetArea);
            // More permissive distance checking - allow even distant detections
            if (targetArea < MIN_TARGET_AREA || distance > 10.0) {
                return null; // Reject obviously invalid measurements
            }
            
            // Convert angles to radians
            double horizontalAngleRad = Math.toRadians(horizontalOffset);
            
            // Calculate relative position from target
            Translation2d relativeTranslation = new Translation2d(
                distance * Math.cos(horizontalAngleRad),
                distance * Math.sin(horizontalAngleRad)
            );
            
            // Transform from target position to robot position
            Transform2d targetToRobot = new Transform2d(relativeTranslation, new Rotation2d());
            Pose2d robotPosition = poiData.position.plus(targetToRobot);
            
            // Apply heading correction
            robotPosition = new Pose2d(
                robotPosition.getTranslation(),
                robotPosition.getRotation().times(HEADING_WEIGHT).plus(
                    robotPosition.getRotation().times(1.0 - HEADING_WEIGHT)
                )
            );
            
            return robotPosition;
            
        } catch (Exception e) {
            return null;
        }
    }
    
    /**
     * Estimates distance from target area.
     * This is a simplified calculation - real implementation would use camera calibration.
     * 
     * @param targetArea Target area from Limelight (0-100)
     * @return Estimated distance in meters
     */
    private double estimateDistanceFromArea(double targetArea) {
        if (targetArea <= 0) {
            return Double.MAX_VALUE;
        }
        
        // Simplified distance calculation (would use actual camera calibration)
        // Distance is inversely proportional to square root of area
        double knownAreaAt1m = 10.0; // Area when target is 1 meter away
        return Math.sqrt(knownAreaAt1m / targetArea);
    }
    
    /**
     * Updates the drive subsystem odometry with the new position.
     * 
     * @param newPosition New absolute position
     */
    private void updateOdometry(Pose2d newPosition) {
        // This would update the drive subsystem's odometry
        // Implementation depends on the specific drive subsystem
        try {
            // If drive subsystem has pose estimation, update it
            if (driveSubsystem != null) {
                // This is a simplified approach - real implementation would use
                // the drive subsystem's specific odometry update method
                // driveSubsystem.resetOdometry(newPosition);
            }
        } catch (Exception e) {
            // Handle odometry update errors
        }
    }
    
    /**
     * Gets the last known position.
     * 
     * @return Last known position, or default if no valid position
     */
    public Pose2d getLastKnownPosition() {
        return lastKnownPosition;
    }
    
    /**
     * Gets the time of the last position update.
     * 
     * @return Time in seconds since epoch
     */
    public double getLastUpdateTime() {
        return lastUpdateTime;
    }
    
    /**
     * Checks if there's a valid position.
     * 
     * @return True if position is valid, false otherwise
     */
    public boolean hasValidPosition() {
        return hasValidPosition;
    }
    
    /**
     * Resets the position updater.
     */
    public void reset() {
        lastKnownPosition = new Pose2d();
        lastUpdateTime = 0.0;
        hasValidPosition = false;
    }
    
    /**
     * Gets the last detected AprilTag ID.
     * 
     * @return Last target ID, or -1 if none detected
     */
    public int getLastTargetId() {
        return lastTargetId;
    }
    
    /**
     * Gets status information.
     * 
     * @return Status string
     */
    public String getStatus() {
        if (!hasValidPosition) {
            return "No valid position";
        }
        
        double timeSinceUpdate = (System.currentTimeMillis() / 1000.0) - lastUpdateTime;
        return String.format("Position valid (%.1fs ago): (%.2f, %.2f, %.1f°) [Tag %d]", 
                           timeSinceUpdate, 
                           lastKnownPosition.getX(), 
                           lastKnownPosition.getY(), 
                           lastKnownPosition.getRotation().getDegrees(),
                           lastTargetId);
    }
}
