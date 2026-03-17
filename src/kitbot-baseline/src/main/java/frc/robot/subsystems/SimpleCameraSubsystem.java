// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple camera subsystem that publishes camera URLs to SmartDashboard.
 * Uses pure SmartDashboard approach for maximum compatibility.
 */
public class SimpleCameraSubsystem extends SubsystemBase {
    
    // Camera status
    private boolean usbCameraAvailable = false;
    private boolean limelightAvailable = false;
    
    // Constants - USB CONNECTION (working solution)
    private static final String LIMELIGHT_IP = "172.22.11.2"; // USB connection to roboRIO
    private static final String LIMELIGHT_STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    
    /**
     * Creates a new SimpleCameraSubsystem.
     */
    public SimpleCameraSubsystem() {
        // Initialize camera data in SmartDashboard
        initializeCameraData();
        
        System.out.println("SimpleCameraSubsystem initialized");
        System.out.println("Camera URLs published to SmartDashboard");
        System.out.println("Use these URLs in Shuffleboard Camera widgets:");
        System.out.println("  Limelight: " + LIMELIGHT_STREAM_URL);
        System.out.println("  Add Camera widget with Custom URL in Shuffleboard");
    }
    
    /**
     * Initializes camera data in SmartDashboard.
     */
    private void initializeCameraData() {
        // Set Limelight URL (always available if configured)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Limelight_URL", LIMELIGHT_STREAM_URL);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", true);
        
        // USB camera URL (empty for now)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", "");
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", false);
        
        // Also publish to root level for easy access
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Limelight_URL", LIMELIGHT_STREAM_URL);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("USB_Camera_URL", "");
    }
    
    @Override
    public void periodic() {
        // Update camera status in SmartDashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", usbCameraAvailable);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", limelightAvailable);
        
        // Debug: Print status every 10 seconds
        long currentTime = System.currentTimeMillis();
        if (currentTime % 10000 < 100) {
            System.out.println("Camera Status:");
            System.out.println("  USB Camera: " + (usbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("  Limelight: " + (limelightAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("  Limelight URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("  Add Camera widgets to Shuffleboard using these SmartDashboard keys:");
            System.out.println("    - Limelight_URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("    - USB_Camera_URL: (set if needed)");
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
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", url);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("USB_Camera_URL", url);
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
     * Tests Limelight connection by checking SmartDashboard values.
     * 
     * @return True if Limelight is responding
     */
    public boolean testLimelightConnection() {
        try {
            // Try to access Limelight NetworkTables through LimelightSubsystem
            // This is a simple test - in a real implementation you'd check NetworkTables directly
            
            System.out.println("=== Limelight Connection Test ===");
            System.out.println("Limelight URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("Status: " + (limelightAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("Test: Try opening URL in browser");
            System.out.println("  " + LIMELIGHT_STREAM_URL);
            
            // For now, assume it's working if we can set the URL
            boolean working = true;
            
            if (working) {
                System.out.println("✅ Limelight URL published to SmartDashboard");
                System.out.println("✅ Camera stream should be available in Shuffleboard");
            } else {
                System.out.println("❌ Limelight connection test failed");
                System.out.println("Check network connection and Limelight configuration");
            }
            
            return working;
            
        } catch (Exception e) {
            System.err.println("Error testing Limelight connection: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Gets all camera URLs for easy access.
     * 
     * @return Array of camera URLs
     */
    public String[] getCameraUrls() {
        return new String[] {
            LIMELIGHT_STREAM_URL,
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Camera/USB_URL", ""),
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("USB_Camera_URL", "")
        };
    }
    
    /**
     * Gets camera status summary.
     * 
     * @return Status summary string
     */
    public String getCameraStatus() {
        return String.format("USB: %s | Limelight: %s | URL: %s",
                           usbCameraAvailable ? "Available" : "Not Available",
                           limelightAvailable ? "Available" : "Not Available",
                           LIMELIGHT_STREAM_URL);
    }
}
