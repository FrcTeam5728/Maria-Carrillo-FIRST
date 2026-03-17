// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Centralized Limelight subsystem that handles all Limelight communication.
 * This replaces multiple programs calling Limelight functions with a single
 * authoritative source for all vision data.
 * 
 * Features:
 * - Single point of contact for Limelight data
 * - Automatic data polling and caching
 * - SmartDashboard integration
 * - Connection monitoring
 * - Target tracking and filtering
 */
public class LimelightSubsystem extends SubsystemBase {
    
    // NetworkTable entries
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tvEntry;    // Valid target (0/1)
    private final NetworkTableEntry txEntry;    // Horizontal offset (-27 to 27)
    private final NetworkTableEntry tyEntry;    // Vertical offset (-20.5 to 20.5)
    private final NetworkTableEntry taEntry;    // Target area (0 to 100)
    private final NetworkTableEntry tidEntry;   // Target ID
    private final NetworkTableEntry tsEntry;    // Timestamp
    private final NetworkTableEntry tlEntry;    // Latency
    
    // Camera feed entries for Shuffleboard
    private final NetworkTableEntry cameraStreamEntry;
    
    // Cached data (updated in periodic)
    private boolean hasTarget = false;
    private double horizontalOffset = 0.0;
    private double verticalOffset = 0.0;
    private double targetArea = 0.0;
    private int targetId = -1;
    private double timestamp = 0.0;
    private double latency = 0.0;
    private boolean isConnected = false;
    
    // Connection tracking
    private long lastUpdateTime = 0;
    private static final long CONNECTION_TIMEOUT_MS = 2000; // 2 seconds
    // Track previous connection state to avoid noisy repeated logs
    private boolean previousConnectedState = true;
    // Minimum interval between connection warnings (ms)
    private static final long CONNECTION_WARNING_INTERVAL_MS = 10000; // 10s
    private long lastConnectionWarning = 0;
    
    // Target filtering
    private static final double MIN_TARGET_AREA = 0.5; // Minimum area to consider valid
    private static final double MAX_HORIZONTAL_OFFSET = 25.0; // Maximum reasonable offset
    
    /**
     * Creates a new LimelightSubsystem.
     */
    public LimelightSubsystem() {
        // Initialize NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Get NetworkTable entries
        tvEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tidEntry = limelightTable.getEntry("tid");
        tsEntry = limelightTable.getEntry("ts");
        tlEntry = limelightTable.getEntry("tl");
        
        // Initialize camera stream entry for Shuffleboard
        cameraStreamEntry = NetworkTableInstance.getDefault().getTable("CameraPublisher")
            .getEntry("LimelightCamera");
        
        System.out.println("LimelightSubsystem initialized");
    }
    
    @Override
    public void periodic() {
        // Update cached data from NetworkTables
        updateLimelightData();
        
        // Update SmartDashboard with current data
        updateSmartDashboard();
        
        // Check connection status
        checkConnection();
    }
    
    /**
     * Updates cached data from Limelight NetworkTables.
     */
    private void updateLimelightData() {
        try {
            // Get raw values from NetworkTables
            double tv = tvEntry.getDouble(0.0);
            double tx = txEntry.getDouble(0.0);
            double ty = tyEntry.getDouble(0.0);
            double ta = taEntry.getDouble(0.0);
            double tid = tidEntry.getDouble(-1.0);
            double ts = tsEntry.getDouble(0.0);
            double tl = tlEntry.getDouble(0.0);
            
            // Apply filtering and validation
            if (tv > 0.5 && ta > MIN_TARGET_AREA && Math.abs(tx) < MAX_HORIZONTAL_OFFSET) {
                // Valid target detected
                hasTarget = true;
                horizontalOffset = tx;
                verticalOffset = ty;
                targetArea = ta;
                targetId = (int) tid;
                timestamp = ts;
                latency = tl;
                lastUpdateTime = System.currentTimeMillis();
            } else {
                // No valid target
                hasTarget = false;
                horizontalOffset = 0.0;
                verticalOffset = 0.0;
                targetArea = 0.0;
                targetId = -1;
            }
            
        } catch (Exception e) {
            System.err.println("Error updating Limelight data: " + e.getMessage());
            // Reset to safe values on error
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

        // Only show connection warnings in real robot mode, not simulation.
        // Print only when the connection state changes or at most once per interval.
        if (!RobotBase.isSimulation()) {
            long now = System.currentTimeMillis();
            if (!nowConnected) {
                boolean shouldWarn = false;
                if (previousConnectedState && timeSinceUpdate > CONNECTION_TIMEOUT_MS * 2) {
                    // just lost connection
                    shouldWarn = true;
                } else if (now - lastConnectionWarning > CONNECTION_WARNING_INTERVAL_MS && timeSinceUpdate > CONNECTION_TIMEOUT_MS * 2) {
                    // rate-limited periodic warning
                    shouldWarn = true;
                }

                if (shouldWarn) {
                    System.err.println("Limelight not responding - check connection");
                    lastConnectionWarning = now;
                }
            }
            previousConnectedState = nowConnected;
        }
    }
    
    /**
     * Updates SmartDashboard with current data.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("Limelight/Connected", isConnected);
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
        SmartDashboard.putNumber("Limelight/HorizontalOffset", horizontalOffset);
        SmartDashboard.putNumber("Limelight/VerticalOffset", verticalOffset);
        SmartDashboard.putNumber("Limelight/TargetArea", targetArea);
        SmartDashboard.putNumber("Limelight/TargetID", targetId);
        SmartDashboard.putNumber("Limelight/Latency", latency);
        
        // Calculate and display distance (simplified)
        if (hasTarget) {
            double distance = calculateDistance();
            SmartDashboard.putNumber("Limelight/Distance", distance);
        } else {
            SmartDashboard.putNumber("Limelight/Distance", 0.0);
        }
        
        // Publish camera stream URL for Shuffleboard
        publishCameraStream();
    }
    
    /**
     * Calculates approximate distance to target.
     * This is a simplified calculation - real implementation would use
     * actual target height and camera mounting angle.
     * 
     * @return Approximate distance in meters
     */
    private double calculateDistance() {
        // Simplified distance calculation
        // In reality, this would use: distance = (targetHeight - cameraHeight) / tan(ty)
        // For now, use target area as a rough proxy
        if (targetArea > 0) {
            return 10.0 / Math.sqrt(targetArea); // Rough approximation
        }
        return 0.0;
    }
    
    /**
     * Publishes camera stream URL for Shuffleboard.
     * Allows viewing Limelight camera feed in Shuffleboard.
     */
    private void publishCameraStream() {
        try {
            // Get Limelight IP (would normally get from NetworkTables or config)
            String limelightIP = "10.57.28.11"; // Default Limelight IP
            
            // Create mjpeg stream URL
            String streamUrl = "http://" + limelightIP + ":5800/stream.mjpg";
            
            // Publish to NetworkTables for Shuffleboard
            cameraStreamEntry.setString(streamUrl);
            
            // Also publish to SmartDashboard for easy access
            SmartDashboard.putString("Limelight/CameraStream", streamUrl);
            
            // Debug: Print stream URL (only once per 5 seconds to avoid spam)
            long currentTime = System.currentTimeMillis();
            if (currentTime % 5000 < 100) {
                System.out.println("Limelight camera stream: " + streamUrl);
                System.out.println("Add this URL to Shuffleboard Camera widget");
            }
            
        } catch (Exception e) {
            System.err.println("Error publishing camera stream: " + e.getMessage());
        }
    }
    
    /**
     * Sets the Limelight pipeline.
     * 
     * @param pipeline Pipeline number (0-9)
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
        System.out.println("Limelight pipeline set to: " + pipeline);
    }
    
    /**
     * Sets the camera mode (vision or driver).
     * 
     * @param driverMode True for driver camera, false for vision processing
     */
    public void setCameraMode(boolean driverMode) {
        limelightTable.getEntry("camMode").setNumber(driverMode ? 1 : 0);
        System.out.println("Limelight camera mode set to: " + (driverMode ? "Driver" : "Vision"));
    }
    
    /**
     * Sets the LED mode.
     * 
     * @param ledMode LED mode (0=use pipeline, 1=off, 2=blink, 3=on)
     */
    public void setLedMode(int ledMode) {
        limelightTable.getEntry("ledMode").setNumber(ledMode);
        System.out.println("Limelight LED mode set to: " + ledMode);
    }
    
    /**
     * Forces the Limelight to take a snapshot.
     */
    public void takeSnapshot() {
        limelightTable.getEntry("snapshot").setNumber(1);
        System.out.println("Limelight snapshot taken");
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
     * @return Target ID (AprilTag number)
     */
    public int getTargetId() {
        return targetId;
    }
    
    /**
     * Gets the distance to target.
     * 
     * @return Approximate distance in meters
     */
    public double getDistance() {
        return calculateDistance();
    }
    
    /**
     * Checks if Limelight is connected.
     * 
     * @return True if connected and responding
     */
    public boolean isConnected() {
        return isConnected;
    }
    
    /**
     * Gets the current latency.
     * 
     * @return Latency in milliseconds
     */
    public double getLatency() {
        return latency;
    }
    
    /**
     * Gets comprehensive target information.
     * 
     * @return Target info object
     */
    public TargetInfo getTargetInfo() {
        return new TargetInfo(hasTarget, targetId, horizontalOffset, verticalOffset, 
                            targetArea, calculateDistance(), latency);
    }
    
    /**
     * Data class for target information.
     */
    public static class TargetInfo {
        public final boolean hasTarget;
        public final int targetId;
        public final double horizontalOffset;
        public final double verticalOffset;
        public final double targetArea;
        public final double distance;
        public final double latency;
        
        public TargetInfo(boolean hasTarget, int targetId, double horizontalOffset, 
                        double verticalOffset, double targetArea, double distance, double latency) {
            this.hasTarget = hasTarget;
            this.targetId = targetId;
            this.horizontalOffset = horizontalOffset;
            this.verticalOffset = verticalOffset;
            this.targetArea = targetArea;
            this.distance = distance;
            this.latency = latency;
        }
        
        @Override
        public String toString() {
            if (!hasTarget) {
                return "No target";
            }
            return String.format("Target %d: %.1fm, H:%.1f° V:%.1f°, Area:%.1f, Latency:%.1fms", 
                                targetId, distance, horizontalOffset, verticalOffset, targetArea, latency);
        }
    }
}
