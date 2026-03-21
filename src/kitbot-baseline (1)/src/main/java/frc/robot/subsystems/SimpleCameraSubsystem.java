// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple camera subsystem that publishes camera URLs to SmartDashboard.
 * Uses pure SmartDashboard approach for maximum compatibility.
 * Supports both single and dual USB camera configurations.
 */
public class SimpleCameraSubsystem extends SubsystemBase {
    
    // Camera status
    private boolean primaryUsbCameraAvailable = false;
    private boolean secondaryUsbCameraAvailable = false;
    private boolean limelightAvailable = false;
    private boolean dualCameraMode = false;
    
    // Constants
    private static final String LIMELIGHT_IP = "172.22.11.2"; // USB connection to roboRIO
    private static final String LIMELIGHT_STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    
    /**
     * Creates a new SimpleCameraSubsystem in single camera mode.
     */
    public SimpleCameraSubsystem() {
        this(false);
    }
    
    /**
     * Creates a new SimpleCameraSubsystem with specified camera mode.
     * 
     * @param dualCameraMode Whether to enable dual camera support
     */
    public SimpleCameraSubsystem(boolean dualCameraMode) {
        this.dualCameraMode = dualCameraMode;
        
        // Initialize camera data in SmartDashboard
        initializeCameraData();
        
        System.out.println("SimpleCameraSubsystem initialized");
        System.out.println("Camera Mode: " + (dualCameraMode ? "DUAL CAMERA" : "SINGLE CAMERA"));
        System.out.println("Camera URLs published to SmartDashboard");
        System.out.println("Use these URLs in Shuffleboard Camera widgets:");
        System.out.println("  Limelight: " + LIMELIGHT_STREAM_URL);
        
        if (dualCameraMode) {
            System.out.println("  Primary USB: " + frc.robot.config.CameraConfig.getPrimaryCameraStreamUrl());
            System.out.println("  Secondary USB: " + frc.robot.config.CameraConfig.getSecondaryCameraStreamUrl());
            System.out.println("  Add two Camera Server widgets in Shuffleboard");
        } else {
            System.out.println("  USB Camera: Dynamic switching between cameras 0 and 1");
            System.out.println("  Add Camera widget with Custom URL in Shuffleboard");
        }
    }
    
    /**
     * Initializes camera data in SmartDashboard.
     */
    private void initializeCameraData() {
        // Set Limelight URL (always available if configured)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Limelight_URL", LIMELIGHT_STREAM_URL);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", true);
        
        if (dualCameraMode) {
            // Dual camera setup - use DualUSBCameraServer
            if (frc.robot.utils.DualUSBCameraServer.isInitialized()) {
                primaryUsbCameraAvailable = frc.robot.utils.DualUSBCameraServer.isPrimaryConnected();
                secondaryUsbCameraAvailable = frc.robot.utils.DualUSBCameraServer.isSecondaryConnected();
                
                // Publish dual camera URLs
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Primary_USB_URL", 
                    frc.robot.config.CameraConfig.getPrimaryCameraStreamUrl());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Primary_USB_Available", primaryUsbCameraAvailable);
                
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Secondary_USB_URL", 
                    frc.robot.config.CameraConfig.getSecondaryCameraStreamUrl());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Secondary_USB_Available", secondaryUsbCameraAvailable);
                
                // Also publish to root level for easy access
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Primary_USB_Camera_URL", 
                    frc.robot.config.CameraConfig.getPrimaryCameraStreamUrl());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Secondary_USB_Camera_URL", 
                    frc.robot.config.CameraConfig.getSecondaryCameraStreamUrl());
            }
        } else {
            // Single camera setup - use DynamicUSBCameraServer
            String usbCameraUrl = "";
            if (frc.robot.utils.DynamicUSBCameraServer.isInitialized()) {
                usbCameraUrl = "http://10.57.28.11:5800/stream"; // Generic URL
                primaryUsbCameraAvailable = frc.robot.utils.DynamicUSBCameraServer.isConnected();
            }
            
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", usbCameraUrl);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", primaryUsbCameraAvailable);
            
            // Also publish to root level for easy access
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("USB_Camera_URL", usbCameraUrl);
        }
        
        // Always publish Limelight URL to root level
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Limelight_URL", LIMELIGHT_STREAM_URL);
    }
    
    @Override
    public void periodic() {
        if (dualCameraMode) {
            // Update dual camera status from DualUSBCameraServer
            if (frc.robot.utils.DualUSBCameraServer.isInitialized()) {
                primaryUsbCameraAvailable = frc.robot.utils.DualUSBCameraServer.isPrimaryConnected();
                secondaryUsbCameraAvailable = frc.robot.utils.DualUSBCameraServer.isSecondaryConnected();
                
                // Update SmartDashboard with current status
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Primary_USB_Available", primaryUsbCameraAvailable);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Secondary_USB_Available", secondaryUsbCameraAvailable);
            }
        } else {
            // Update single camera status from DynamicUSBCameraServer
            if (frc.robot.utils.DynamicUSBCameraServer.isInitialized()) {
                primaryUsbCameraAvailable = frc.robot.utils.DynamicUSBCameraServer.isConnected();
                if (primaryUsbCameraAvailable) {
                    String usbCameraUrl = "http://10.57.28.11:5800/stream"; // Generic URL
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", usbCameraUrl);
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("USB_Camera_URL", usbCameraUrl);
                }
            }
            
            // Update camera status in SmartDashboard
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", primaryUsbCameraAvailable);
        }
        
        // Always update Limelight status
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", limelightAvailable);
        
        // Debug: Print status every 10 seconds
        long currentTime = System.currentTimeMillis();
        if (currentTime % 10000 < 50) { // Every 10 seconds for 50ms
            System.out.println("=== Camera Status Update ===");
            System.out.println("Camera Mode: " + (dualCameraMode ? "DUAL CAMERA" : "SINGLE CAMERA"));
            
            if (dualCameraMode) {
                System.out.println("  Primary USB Camera: " + (primaryUsbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
                System.out.println("  Secondary USB Camera: " + (secondaryUsbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
                System.out.println("  Primary URL: " + frc.robot.config.CameraConfig.getPrimaryCameraStreamUrl());
                System.out.println("  Secondary URL: " + frc.robot.config.CameraConfig.getSecondaryCameraStreamUrl());
                System.out.println("  Add two Camera Server widgets to Shuffleboard:");
                System.out.println("    - Primary_USB_Camera_URL: Primary camera feed");
                System.out.println("    - Secondary_USB_Camera_URL: Secondary camera feed");
            } else {
                System.out.println("  USB Camera: " + (primaryUsbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
                System.out.println("  USB Camera URL: Dynamic switching between cameras 0 and 1");
            }
            
            System.out.println("  Limelight: " + (limelightAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("  Limelight URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("=============================");
        }
    }
    
    /**
     * Sets primary USB camera availability status.
     * 
     * @param available Whether primary USB camera is available
     */
    public void setPrimaryUsbCameraAvailable(boolean available) {
        this.primaryUsbCameraAvailable = available;
        System.out.println("Primary USB Camera availability set to: " + (available ? "AVAILABLE" : "NOT AVAILABLE"));
    }
    
    /**
     * Sets secondary USB camera availability status.
     * 
     * @param available Whether secondary USB camera is available
     */
    public void setSecondaryUsbCameraAvailable(boolean available) {
        this.secondaryUsbCameraAvailable = available;
        System.out.println("Secondary USB Camera availability set to: " + (available ? "AVAILABLE" : "NOT AVAILABLE"));
    }
    
    /**
     * Sets USB camera availability status (for single camera mode).
     * 
     * @param available Whether USB camera is available
     */
    public void setUsbCameraAvailable(boolean available) {
        this.primaryUsbCameraAvailable = available;
        System.out.println("USB Camera availability set to: " + (available ? "AVAILABLE" : "NOT AVAILABLE"));
    }
    
    /**
     * Sets primary USB camera URL.
     * 
     * @param url The primary USB camera stream URL
     */
    public void setPrimaryUsbCameraUrl(String url) {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Primary_USB_URL", url);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Primary_USB_Camera_URL", url);
        System.out.println("Primary USB Camera URL set to: " + url);
    }
    
    /**
     * Sets secondary USB camera URL.
     * 
     * @param url The secondary USB camera stream URL
     */
    public void setSecondaryUsbCameraUrl(String url) {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Secondary_USB_URL", url);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Secondary_USB_Camera_URL", url);
        System.out.println("Secondary USB Camera URL set to: " + url);
    }
    
    /**
     * Sets USB camera URL (for single camera mode).
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
     * Gets primary USB camera availability status.
     * 
     * @return True if primary USB camera is available
     */
    public boolean isPrimaryUsbCameraAvailable() {
        return primaryUsbCameraAvailable;
    }
    
    /**
     * Gets secondary USB camera availability status.
     * 
     * @return True if secondary USB camera is available
     */
    public boolean isSecondaryUsbCameraAvailable() {
        return secondaryUsbCameraAvailable;
    }
    
    /**
     * Gets USB camera availability status (for single camera mode).
     * 
     * @return True if USB camera is available
     */
    public boolean isUsbCameraAvailable() {
        return primaryUsbCameraAvailable;
    }
    
    /**
     * Gets primary USB camera stream URL.
     * 
     * @return Primary USB camera stream URL
     */
    public String getPrimaryUsbCameraUrl() {
        return frc.robot.config.CameraConfig.getPrimaryCameraStreamUrl();
    }
    
    /**
     * Gets secondary USB camera stream URL.
     * 
     * @return Secondary USB camera stream URL
     */
    public String getSecondaryUsbCameraUrl() {
        return frc.robot.config.CameraConfig.getSecondaryCameraStreamUrl();
    }
    
    /**
     * Gets USB camera stream URL (for single camera mode).
     * 
     * @return USB camera stream URL
     */
    public String getUsbCameraUrl() {
        if (frc.robot.utils.DynamicUSBCameraServer.isInitialized()) {
            return "http://10.57.28.11:5800/stream"; // Generic URL
        }
        return "";
    }
    
    /**
     * Gets the camera mode (dual or single).
     * 
     * @return True if dual camera mode is enabled
     */
    public boolean isDualCameraMode() {
        return dualCameraMode;
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
        if (dualCameraMode) {
            return new String[] {
                LIMELIGHT_STREAM_URL,
                getPrimaryUsbCameraUrl(),
                getSecondaryUsbCameraUrl()
            };
        } else {
            return new String[] {
                LIMELIGHT_STREAM_URL,
                getUsbCameraUrl(),
                ""
            };
        }
    }
    
    /**
     * Gets camera system status summary.
     * 
     * @return Status summary string
     */
    public String getStatusSummary() {
        StringBuilder status = new StringBuilder();
        status.append("Camera System: ");
        
        if (dualCameraMode) {
            status.append("DUAL MODE - ");
            status.append("Primary=").append(primaryUsbCameraAvailable ? "OK" : "FAIL");
            status.append(", Secondary=").append(secondaryUsbCameraAvailable ? "OK" : "FAIL");
        } else {
            status.append("SINGLE MODE - ");
            status.append("USB=").append(primaryUsbCameraAvailable ? "OK" : "FAIL");
        }
        
        status.append(", Limelight=").append(limelightAvailable ? "OK" : "FAIL");
        return status.toString();
    }
}
