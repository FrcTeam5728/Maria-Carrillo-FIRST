// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * NetworkTables-based camera subsystem.
 * Publishes camera stream URLs and status to NetworkTables for Shuffleboard.
 * Uses pure NetworkTables approach instead of complex CameraServer.
 */
public class NetworkTablesCameraSubsystem extends SubsystemBase {
    
    // NetworkTables entries
    private final NetworkTable cameraTable;
    private final NetworkTable.Entry usbCameraUrlEntry;
    private final NetworkTableEntry limelightUrlEntry;
    private final NetworkTable.Entry usbCameraStatusEntry;
    private final NetworkTable.Entry limelightStatusEntry;
    
    // Camera status
    private boolean usbCameraAvailable = false;
    private boolean limelightAvailable = false;
    
    // Constants
    private static final String LIMELIGHT_IP = "10.57.28.11";
    private static final String LIMELIGHT_STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    
    /**
     * Creates a new NetworkTablesCameraSubsystem.
     */
    public NetworkTablesCameraSubsystem() {
        // Create camera table
        cameraTable = NetworkTableInstance.getDefault().getTable("Camera");
        
        // Initialize NetworkTable entries
        usbCameraUrlEntry = cameraTable.getEntry("USB_Camera_URL");
        limelightUrlEntry = cameraTable.getEntry("Limelight_URL");
        usbCameraStatusEntry = cameraTable.getEntry("USB_Camera_Available");
        limelightStatusEntry = cameraTable.getEntry("Limelight_Available");
        
        // Set initial values
        initializeCameraData();
        
        System.out.println("NetworkTablesCameraSubsystem initialized");
        System.out.println("Camera URLs published to NetworkTables");
        System.out.println("Use these URLs in Shuffleboard Camera widgets:");
        System.out.println("  USB: " + usbCameraUrlEntry.getString(""));
        System.out.println("  Limelight: " + limelightUrlEntry.getString(""));
    }
    
    /**
     * Initializes camera data in NetworkTables.
     */
    private void initializeCameraData() {
        // Set Limelight URL (always available if configured)
        limelightUrlEntry.setString(LIMELIGHT_STREAM_URL);
        limelightStatusEntry.setBoolean(true); // Assume available until proven otherwise
        
        // USB camera URL (would be configured based on actual camera)
        // For now, leave empty - user can add manually if needed
        usbCameraUrlEntry.setString("");
        usbCameraStatusEntry.setBoolean(false);
        
        // Also publish to SmartDashboard for compatibility
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", "");
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Limelight_URL", LIMELIGHT_STREAM_URL);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", false);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", true);
    }
    
    @Override
    public void periodic() {
        // Update camera status (could add connection checking here)
        updateCameraStatus();
        
        // Publish status to NetworkTables
        usbCameraStatusEntry.setBoolean(usbCameraAvailable);
        limelightStatusEntry.setBoolean(limelightAvailable);
        
        // Also publish to SmartDashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", usbCameraAvailable);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", limelightAvailable);
    }
    
    /**
     * Updates camera status (placeholder for future connection checking).
     */
    private void updateCameraStatus() {
        // For USB camera, we could check if camera is connected
        // For now, assume it's not available unless explicitly set
        
        // For Limelight, we could ping the IP or check NetworkTables
        // For now, assume it's available
        limelightAvailable = true;
        
        // Debug: Print status every 10 seconds
        long currentTime = System.currentTimeMillis();
        if (currentTime % 10000 < 100) {
            System.out.println("Camera Status:");
            System.out.println("  USB Camera: " + (usbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("  Limelight: " + (limelightAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("  Limelight URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("  Add Camera widgets to Shuffleboard using these URLs");
        }
    }
    
    /**
     * Sets USB camera availability status.
     * Call this if you implement USB camera detection.
     * 
     * @param available Whether USB camera is available
     */
    public void setUsbCameraAvailable(boolean available) {
        this.usbCameraAvailable = available;
        System.out.println("USB Camera availability set to: " + (available ? "AVAILABLE" : "NOT AVAILABLE"));
    }
    
    /**
     * Sets USB camera URL.
     * 
     * @param url The USB camera stream URL
     */
    public void setUsbCameraUrl(String url) {
        usbCameraUrlEntry.setString(url);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", url);
        System.out.println("USB Camera URL set to: " + url);
    }
    
    /**
     * Gets the Limelight stream URL.
     * 
     * @return Limelight MJPEG stream URL
     */
    public String getLimelightStreamUrl() {
        return LIMELIGHT_STREAM_URL;
    }
    
    /**
     * Gets USB camera availability status.
     * 
     * @return True if USB camera is available
     */
    public boolean isUsbCameraAvailable() {
        return usbCameraAvailable;
    }
    
    /**
     * Gets Limelight availability status.
     * 
     * @return True if Limelight is available
     */
    public boolean isLimelightAvailable() {
        return limelightAvailable;
    }
    
    /**
     * Gets camera table for advanced configuration.
     * 
     * @return Camera NetworkTable
     */
    public NetworkTable getCameraTable() {
        return cameraTable;
    }
    
    /**
     * Tests Limelight connection by checking NetworkTables.
     * 
     * @return True if Limelight is responding
     */
    public boolean testLimelightConnection() {
        try {
            // Try to access Limelight NetworkTables
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            double tv = limelightTable.getEntry("tv").getDouble(-1.0);
            
            // If we get any value (not the error value), NetworkTables is working
            boolean networkTablesWorking = tv != -1.0;
            
            System.out.println("=== Limelight Connection Test ===");
            System.out.println("NetworkTables access: " + (networkTablesWorking ? "WORKING" : "FAILED"));
            System.out.println("Limelight URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("Status: " + (limelightAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            
            if (networkTablesWorking) {
                System.out.println("✅ Limelight NetworkTables working");
                System.out.println("✅ Camera stream should be available");
            } else {
                System.out.println("❌ Limelight NetworkTables not responding");
                System.out.println("Check network connection and Limelight configuration");
            }
            
            return networkTablesWorking;
            
        } catch (Exception e) {
            System.err.println("Error testing Limelight connection: " + e.getMessage());
            return false;
        }
    }
}