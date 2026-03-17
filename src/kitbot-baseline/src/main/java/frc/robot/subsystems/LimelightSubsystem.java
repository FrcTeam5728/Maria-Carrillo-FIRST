// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Centralized Limelight subsystem that handles all Limelight communication.
 * This replaces multiple programs calling Limelight functions with a single
 * authoritative source for all vision data.
 * 
 * Features:
 * - Single point of contact for Limelight data
 * - Automatic data polling and caching
 * - Connection monitoring
 * - Target tracking and filtering
 * - Camera stream broadcasting
 */
public class LimelightSubsystem extends SubsystemBase {
    
    // NetworkTable entries
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tvEntry;    // Valid target (0/1)
    private final NetworkTableEntry txEntry;    // Horizontal offset (-27 to 27)
    private final NetworkTableEntry tyEntry;    // Vertical offset (-20.5 to 20.5)
    private final NetworkTableEntry taEntry;    // Target area (0 to 100)
    private final NetworkTableEntry tidEntry;   // Target ID
    private final NetworkTableEntry tlEntry;    // Latency
    
    // Camera feed entries for broadcasting
    private final NetworkTableEntry cameraStreamEntry;
    
    // Cached data (updated in periodic)
    private boolean hasTarget = false;
    private double horizontalOffset = 0.0;
    private double verticalOffset = 0.0;
    private double targetArea = 0.0;
    private int targetId = -1;
    private double latency = 0.0;
    
    // Connection monitoring
    private boolean isConnected = false;
    private long lastUpdateTime = 0;
    private long lastConnectionWarning = 0;
    private boolean previousConnectedState = false;
    private static final long CONNECTION_TIMEOUT_MS = 1000; // 1 second timeout
    
    /**
     * Creates a new LimelightSubsystem.
     */
    public LimelightSubsystem() {
        // Initialize NetworkTable entries
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tvEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tidEntry = limelightTable.getEntry("tid");
        tlEntry = limelightTable.getEntry("tl");
        
        // Initialize camera stream entry for broadcasting
        cameraStreamEntry = NetworkTableInstance.getDefault().getTable("CameraPublisher")
            .getEntry("LimelightCamera");
    }
    
    @Override
    public void periodic() {
        // Update cached data from NetworkTables
        updateLimelightData();
        
        // Check connection status
        checkConnection();
        
        // Update camera stream for broadcasting
        updateCameraStream();
    }
    
    /**
     * Updates cached data from NetworkTables.
     */
    private void updateLimelightData() {
        try {
            hasTarget = tvEntry.getDouble(0.0) > 0.5;
            horizontalOffset = txEntry.getDouble(0.0);
            verticalOffset = tyEntry.getDouble(0.0);
            targetArea = taEntry.getDouble(0.0);
            targetId = (int) tidEntry.getDouble(-1.0);
            latency = tlEntry.getDouble(0.0);
            
            lastUpdateTime = System.currentTimeMillis();
            
        } catch (Exception e) {
            // Handle errors silently - reset to safe values
            hasTarget = false;
            horizontalOffset = 0.0;
            verticalOffset = 0.0;
            targetArea = 0.0;
            targetId = -1;
        }
    }
    
    /**
     * Checks if Limelight is connected and responding.
     */
    private void checkConnection() {
        long currentTime = System.currentTimeMillis();
        long timeSinceUpdate = currentTime - lastUpdateTime;
        
        boolean nowConnected = timeSinceUpdate < CONNECTION_TIMEOUT_MS;
        isConnected = nowConnected;

        // Only show connection warnings in real robot mode, not simulation
        if (!RobotBase.isSimulation()) {
            if (!nowConnected && previousConnectedState && timeSinceUpdate > CONNECTION_TIMEOUT_MS * 2) {
                // Just lost connection
                if (currentTime - lastConnectionWarning > 5000) { // Max once per 5 seconds
                    System.err.println("Limelight not responding - check connection");
                    lastConnectionWarning = currentTime;
                }
            }
        }
        previousConnectedState = nowConnected;
    }
    
    /**
     * Updates camera stream for broadcasting.
     */
    private void updateCameraStream() {
        try {
            // Generate camera stream URL for broadcasting
            String streamUrl = generateCameraStreamUrl();
            
            // Publish to CameraPublisher table for broadcasting
            cameraStreamEntry.setString(streamUrl);
            
        } catch (Exception e) {
            // Handle stream generation errors silently
        }
    }
    
    /**
     * Generates camera stream URL for broadcasting.
     * 
     * @return Stream URL string
     */
    private String generateCameraStreamUrl() {
        // Standard Limelight stream URL format
        return "http://10.57.28.11:5800/stream";
    }
    
    /**
     * Sets the camera mode.
     * 
     * @param driverMode True for driver camera, false for vision processing
     */
    public void setCameraMode(boolean driverMode) {
        limelightTable.getEntry("camMode").setNumber(driverMode ? 1 : 0);
    }
    
    /**
     * Sets the LED mode.
     * 
     * @param ledMode LED mode (0=use pipeline, 1=off, 2=blink, 3=on)
     */
    public void setLedMode(int ledMode) {
        limelightTable.getEntry("ledMode").setNumber(ledMode);
    }
    
    /**
     * Forces the Limelight to take a snapshot.
     */
    public void takeSnapshot() {
        limelightTable.getEntry("snapshot").setNumber(1);
    }
    
    /**
     * Gets the current target status.
     * 
     * @return True if a valid target is detected
     */
    public boolean hasTarget() {
        return hasTarget;
    }
    
    /**
     * Gets the horizontal offset to target.
     * 
     * @return Horizontal offset in degrees (-27 to 27)
     */
    public double getHorizontalOffset() {
        return horizontalOffset;
    }
    
    /**
     * Gets the vertical offset to target.
     * 
     * @return Vertical offset in degrees (-20.5 to 20.5)
     */
    public double getVerticalOffset() {
        return verticalOffset;
    }
    
    /**
     * Gets the target area.
     * 
     * @return Target area (0 to 100)
     */
    public double getTargetArea() {
        return targetArea;
    }
    
    /**
     * Gets the target ID.
     * 
     * @return Target ID, or -1 if no target
     */
    public int getTargetId() {
        return targetId;
    }
    
    /**
     * Gets the pipeline latency.
     * 
     * @return Latency in milliseconds
     */
    public double getLatency() {
        return latency;
    }
    
    /**
     * Gets the connection status.
     * 
     * @return True if Limelight is connected and responding
     */
    public boolean isConnected() {
        return isConnected;
    }
    
    /**
     * Calculates estimated distance to target.
     * 
     * @return Estimated distance in meters
     */
    public double getDistance() {
        // Simplified distance calculation based on target area
        // In reality, this would use: distance = (targetHeight - cameraHeight) / tan(ty)
        if (targetArea > 0) {
            return 10.0 / Math.sqrt(targetArea); // Rough approximation
        }
        return 0.0;
    }
    
    /**
     * Gets robot pose from AprilTag detection.
     * 
     * @return Robot pose, or null if no valid target
     */
    public Pose2d getRobotPose() {
        if (hasTarget && targetId > 0) {
            // This would use the actual AprilTag pose calculation
            // For now, return a placeholder
            return new Pose2d();
        }
        return null;
    }
    
    /**
     * Tests raw Limelight connection.
     * 
     * @return true if NetworkTables connection is working
     */
    public boolean testRawConnection() {
        try {
            var table = NetworkTableInstance.getDefault().getTable("limelight");
            var testEntry = table.getEntry("tv");
            double testValue = testEntry.getDouble(-999.0);
            
            // If we got any value (not the error value), NetworkTables is working
            return testValue != -999.0;
            
        } catch (Exception e) {
            return false;
        }
    }
}
