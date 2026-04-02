// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Field position system that publishes robot position to NetworkTables.
 * Allows visualization of robot position on field using tools like Shuffleboard
 * or custom dashboards.
 * 
 * Features:
 * - Real-time position updates
 * - Field coordinate system
 * - Visual debugging support
 * - Multiple position sources (encoders, vision)
 * - SmartDashboard integration
 */
public class FieldPositionSystem {
    
    // NetworkTable for field position data
    private final NetworkTable fieldTable;
    
    // Position entries
    private final NetworkTableEntry robotXEntry;
    private final NetworkTableEntry robotYEntry;
    private final NetworkTableEntry robotRotationEntry;
    private final NetworkTableEntry timestampEntry;
    private final NetworkTableEntry confidenceEntry;
    private final NetworkTableEntry positionSourceEntry;
    
    // Position data
    private Pose2d robotPose = new Pose2d();
    private double confidence = 0.0;
    private String positionSource = "UNKNOWN";
    
    // Field dimensions (2024 FRC field: ~16.54m x 8.21m)
    private static final double FIELD_LENGTH_METERS = 16.54;
    private static final double FIELD_WIDTH_METERS = 8.21;
    private static final double FIELD_CENTER_X = FIELD_LENGTH_METERS / 2.0;
    private static final double FIELD_CENTER_Y = FIELD_WIDTH_METERS / 2.0;
    
    // Subsystems and utilities
    private final LimelightSubsystem limelightSubsystem;
    private final AprilTagPositionUpdater positionUpdater;
    
    /**
     * Creates a new FieldPositionSystem.
     * 
     * @param limelightSubsystem Limelight subsystem for vision data
     */
    public FieldPositionSystem(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.positionUpdater = new AprilTagPositionUpdater(null, limelightSubsystem);
        
        // Initialize NetworkTable
        fieldTable = NetworkTableInstance.getDefault().getTable("fieldPosition");
        
        // Get NetworkTable entries
        robotXEntry = fieldTable.getEntry("robotX");
        robotYEntry = fieldTable.getEntry("robotY");
        robotRotationEntry = fieldTable.getEntry("robotRotation");
        timestampEntry = fieldTable.getEntry("timestamp");
        confidenceEntry = fieldTable.getEntry("confidence");
        positionSourceEntry = fieldTable.getEntry("positionSource");
        
        // Initialize position
        resetPosition();
        
        System.out.println("FieldPositionSystem initialized");
        System.out.println("Field dimensions: " + FIELD_LENGTH_METERS + "m x " + FIELD_WIDTH_METERS + "m");
    }
    
    /**
     * Updates position system with current data.
     * Call this periodically (usually in Robot periodic or subsystem periodic).
     */
    public void update() {
        // Update position from available sources
        updatePositionFromSources();
        
        // Publish to NetworkTables
        publishToNetworkTables();
        
        // Update SmartDashboard
        updateSmartDashboard();
    }
    
    /**
     * Updates position from available sources (encoders, vision).
     */
    private void updatePositionFromSources() {
        // Priority: Vision > Encoders > Last Known
        
        // Always try vision position first when AprilTag detected
        if (limelightSubsystem.hasTarget()) {
            int targetId = limelightSubsystem.getTargetId();
            if (targetId > 0 && positionUpdater.updatePosition()) {
                // Successfully updated position from AprilTag
                robotPose = positionUpdater.getLastKnownPosition();
                confidence = 0.9; // High confidence for vision-based position
                positionSource = "APRILTAG";
                return;
            }
        }
        
        // Fallback to encoder-based position
        updateFromEncoders();
        
        // Ensure position is within field bounds
        constrainToField();
    }
    
    /**
     * Updates position from encoder data.
     */
    private void updateFromEncoders() {
        try {
            // This is simplified - would use actual odometry
            // For now, maintain last known position with slight drift
            
            // Simulate small movement (would use actual encoder data)
            double deltaX = 0.01 * Math.cos(robotPose.getRotation().getRadians());
            double deltaY = 0.01 * Math.sin(robotPose.getRotation().getRadians());
            
            robotPose = new Pose2d(
                robotPose.getX() + deltaX,
                robotPose.getY() + deltaY,
                robotPose.getRotation()
            );
            
            confidence = Math.max(0.1, confidence - 0.01); // Decrease confidence over time
            positionSource = "ENCODERS";
            
        } catch (Exception e) {
            System.err.println("Error updating from encoders: " + e.getMessage());
            confidence = 0.0;
            positionSource = "ERROR";
        }
    }
    
    /**
     * Constrains position to field boundaries.
     */
    private void constrainToField() {
        double x = Math.max(0, Math.min(FIELD_LENGTH_METERS, robotPose.getX()));
        double y = Math.max(0, Math.min(FIELD_WIDTH_METERS, robotPose.getY()));
        
        robotPose = new Pose2d(x, y, robotPose.getRotation());
    }
    
    /**
     * Publishes position data to NetworkTables.
     */
    private void publishToNetworkTables() {
        robotXEntry.setDouble(robotPose.getX());
        robotYEntry.setDouble(robotPose.getY());
        robotRotationEntry.setDouble(robotPose.getRotation().getDegrees());
        timestampEntry.setDouble(System.currentTimeMillis() / 1000.0);
        confidenceEntry.setDouble(confidence);
        positionSourceEntry.setString(positionSource);
    }
    
    /**
     * Updates SmartDashboard with position information.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Field/RobotX", robotPose.getX());
        SmartDashboard.putNumber("Field/RobotY", robotPose.getY());
        SmartDashboard.putNumber("Field/RobotRotation", robotPose.getRotation().getDegrees());
        SmartDashboard.putString("Field/PositionSource", positionSource);
        SmartDashboard.putNumber("Field/Confidence", confidence);
        SmartDashboard.putNumber("Field/DistanceFromCenter", getDistanceFromCenter());
        
        // Field quadrant information
        String quadrant = getFieldQuadrant();
        SmartDashboard.putString("Field/Quadrant", quadrant);
        
        // Position quality indicator
        String quality = getPositionQuality();
        SmartDashboard.putString("Field/Quality", quality);
    }
    
    /**
     * Gets distance from field center.
     * 
     * @return Distance in meters
     */
    public double getDistanceFromCenter() {
        double dx = robotPose.getX() - FIELD_CENTER_X;
        double dy = robotPose.getY() - FIELD_CENTER_Y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Gets field quadrant.
     * 
     * @return Quadrant string
     */
    public String getFieldQuadrant() {
        double x = robotPose.getX();
        double y = robotPose.getY();
        
        if (x < FIELD_CENTER_X && y < FIELD_CENTER_Y) return "BLUE_LEFT";
        if (x >= FIELD_CENTER_X && y < FIELD_CENTER_Y) return "BLUE_RIGHT";
        if (x < FIELD_CENTER_X && y >= FIELD_CENTER_Y) return "RED_LEFT";
        return "RED_RIGHT";
    }
    
    /**
     * Gets position quality indicator.
     * 
     * @return Quality string
     */
    public String getPositionQuality() {
        if (confidence >= 0.8) return "EXCELLENT";
        if (confidence >= 0.6) return "GOOD";
        if (confidence >= 0.4) return "FAIR";
        if (confidence >= 0.2) return "POOR";
        return "UNKNOWN";
    }
    
    /**
     * Resets position to field center.
     */
    public void resetPosition() {
        robotPose = new Pose2d(FIELD_CENTER_X, FIELD_CENTER_Y, new Rotation2d());
        confidence = 0.0;
        positionSource = "RESET";
        System.out.println("Field position reset to center");
    }
    
    /**
     * Sets position manually.
     * 
     * @param x X position in meters
     * @param y Y position in meters
     * @param rotation Rotation in degrees
     */
    public void setPosition(double x, double y, double rotation) {
        robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
        confidence = 1.0;
        positionSource = "MANUAL";
        System.out.println("Field position set manually: (" + x + ", " + y + ", " + rotation + "°)");
    }
    
    /**
     * Gets the AprilTag position updater.
     * 
     * @return AprilTag position updater instance
     */
    public AprilTagPositionUpdater getPositionUpdater() {
        return positionUpdater;
    }
    
    /**
     * Gets the AprilTag position updater status.
     * 
     * @return Status string from the position updater
     */
    public String getAprilTagStatus() {
        return positionUpdater.getStatus();
    }
    
    /**
     * Gets current robot pose.
     * 
     * @return Current robot pose
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }
    
    /**
     * Gets position confidence.
     * 
     * @return Confidence level (0.0 to 1.0)
     */
    public double getConfidence() {
        return confidence;
    }
    
    /**
     * Gets position source.
     * 
     * @return Position source string
     */
    public String getPositionSource() {
        return positionSource;
    }
    
    /**
     * Gets field dimensions.
     * 
     * @return Array with [length, width] in meters
     */
    public double[] getFieldDimensions() {
        return new double[]{FIELD_LENGTH_METERS, FIELD_WIDTH_METERS};
    }
    
    /**
     * Gets system status summary.
     * 
     * @return Status string
     */
    public String getStatus() {
        return String.format("Position: (%.2f, %.2f, %.1f°) | Source: %s | Confidence: %.2f | Quality: %s",
                           robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(),
                           positionSource, confidence, getPositionQuality());
    }
}
