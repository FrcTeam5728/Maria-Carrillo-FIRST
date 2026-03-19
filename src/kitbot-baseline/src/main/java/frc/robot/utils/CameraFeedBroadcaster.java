// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Utility class for broadcasting Limelight camera feed.
 * Sets up camera stream for Shuffleboard and driver station viewing.
 */
public class CameraFeedBroadcaster extends SubsystemBase {
    
    private final NetworkTable cameraPublisherTable;
    private final NetworkTableEntry streamEntry;
    private final String limelightIpAddress;
    private final int streamPort;
    
    /**
     * Creates a new CameraFeedBroadcaster.
     * 
     * @param limelightIpAddress IP address of the Limelight
     * @param streamPort Port for the camera stream (usually 5800)
     */
    public CameraFeedBroadcaster(String limelightIpAddress, int streamPort) {
        this.limelightIpAddress = limelightIpAddress;
        this.streamPort = streamPort;
        
        // Get CameraPublisher table for broadcasting
        cameraPublisherTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
        streamEntry = cameraPublisherTable.getEntry("LimelightCamera");
    }
    
    /**
     * Creates a new CameraFeedBroadcaster with default settings.
     */
    public CameraFeedBroadcaster() {
        this("10.57.28.11", 5800); // Default team IP and port
    }
    
    @Override
    public void periodic() {
        // Only publish stream URL periodically to avoid storage issues
        long currentTime = System.currentTimeMillis();
        if (currentTime % 5000 < 20) { // Every 5 seconds for 20ms
            publishCameraStream();
        }
    }
    
    /**
     * Publishes the camera stream URL to NetworkTables.
     */
    private void publishCameraStream() {
        try {
            String streamUrl = generateStreamUrl();
            streamEntry.setString(streamUrl);
            
            // Also publish to subtable for Shuffleboard compatibility
            NetworkTable limelightCameraTable = cameraPublisherTable.getSubTable("LimelightCamera");
            limelightCameraTable.getEntry("source").setString(streamUrl);
            limelightCameraTable.getEntry("description").setString("Limelight Camera Feed");
            
        } catch (Exception e) {
            // Handle errors silently to avoid disrupting robot operation
        }
    }
    
    /**
     * Generates the camera stream URL.
     * 
     * @return Stream URL string
     */
    private String generateStreamUrl() {
        return String.format("http://%s:%d/stream", limelightIpAddress, streamPort);
    }
    
    /**
     * Gets the current stream URL.
     * 
     * @return Stream URL string
     */
    public String getStreamUrl() {
        return generateStreamUrl();
    }
    
    /**
     * Sets the Limelight IP address.
     * 
     * @param ipAddress New IP address
     */
    public void setLimelightIpAddress(String ipAddress) {
        // Note: This would require recreating the NetworkTable entries
        // For now, this is just a placeholder for future implementation
    }
    
    /**
     * Enables/disables camera stream broadcasting.
     * 
     * @param enabled True to enable broadcasting
     */
    public void setBroadcastingEnabled(boolean enabled) {
        if (enabled) {
            publishCameraStream();
        } else {
            // Clear the stream entry when disabled
            streamEntry.setString("");
        }
    }
}
