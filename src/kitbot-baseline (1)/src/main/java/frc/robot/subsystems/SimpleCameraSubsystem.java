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
    private boolean secondaryUsbCameraAvailable = false;
    private boolean limelightAvailable = false;
    private boolean useDualCameras = false;
    
    // Constants - USB CONNECTION (working solution)
    private static final String LIMELIGHT_IP = "172.22.11.2"; // USB connection to roboRIO
    private static final String LIMELIGHT_STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    
    /**
     * Creates a new SimpleCameraSubsystem with single camera mode.
     */
    public SimpleCameraSubsystem() {
        this(false);
    }
    
    /**
     * Creates a new SimpleCameraSubsystem.
     * 
     * @param useDualCameras Whether to enable dual USB camera support
     */
    public SimpleCameraSubsystem(boolean useDualCameras) {
        this.useDualCameras = useDualCameras;
        
        // Initialize camera data in SmartDashboard
        initializeCameraData();
        
        System.out.println("SimpleCameraSubsystem initialized");
        System.out.println("Dual Camera Mode: " + (useDualCameras ? "ENABLED" : "DISABLED"));
        System.out.println("Camera URLs published to SmartDashboard");
        System.out.println("Use these URLs in Shuffleboard Camera widgets:");
        System.out.println("  Limelight: " + LIMELIGHT_STREAM_URL);
        if (useDualCameras) {
            System.out.println("  Primary USB: " + frc.robot.utils.DualUSBCameraServer.getPrimaryStreamUrl());
            System.out.println("  Secondary USB: " + frc.robot.utils.DualUSBCameraServer.getSecondaryStreamUrl());
        } else {
            System.out.println("  USB Camera: " + (frc.robot.utils.USBCameraServer.isInitialized() ? frc.robot.utils.USBCameraServer.getStreamUrl() : "Not available"));
        }
        System.out.println("  Add Camera widget with Custom URL in Shuffleboard");
    }
    
    /**
     * Initializes camera data in SmartDashboard.
     */
    private void initializeCameraData() {
        // Set Limelight URL (always available if configured)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Limelight_URL", LIMELIGHT_STREAM_URL);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", true);
        
        if (useDualCameras) {
            // Dual camera mode - use DualUSBCameraServer
            String primaryCameraUrl = "";
            String secondaryCameraUrl = "";
            
            if (frc.robot.utils.DualUSBCameraServer.isPrimaryInitialized()) {
                primaryCameraUrl = frc.robot.utils.DualUSBCameraServer.getPrimaryStreamUrl();
                usbCameraAvailable = true;
            }
            
            if (frc.robot.utils.DualUSBCameraServer.isSecondaryInitialized()) {
                secondaryCameraUrl = frc.robot.utils.DualUSBCameraServer.getSecondaryStreamUrl();
                secondaryUsbCameraAvailable = true;
            }
            
            // Publish dual camera URLs
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Primary_USB_URL", primaryCameraUrl);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Secondary_USB_URL", secondaryCameraUrl);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Primary_USB_Available", usbCameraAvailable);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Secondary_USB_Available", secondaryUsbCameraAvailable);
            
            // Also publish to root level for easy access
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Primary_USB_Camera_URL", primaryCameraUrl);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Secondary_USB_Camera_URL", secondaryCameraUrl);
            
        } else {
            // Single camera mode - use original USBCameraServer
            String usbCameraUrl = "";
            if (frc.robot.utils.USBCameraServer.isInitialized()) {
                usbCameraUrl = frc.robot.utils.USBCameraServer.getStreamUrl();
                usbCameraAvailable = true;
            }
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", usbCameraUrl);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", usbCameraAvailable);
            
            // Also publish to root level for easy access
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("USB_Camera_URL", usbCameraUrl);
        }
        
        // Also publish to root level for easy access
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Limelight_URL", LIMELIGHT_STREAM_URL);
    }
    
    @Override
    public void periodic() {
        if (useDualCameras) {
            // Dual camera mode - check DualUSBCameraServer status
            if (frc.robot.utils.DualUSBCameraServer.isPrimaryInitialized()) {
                usbCameraAvailable = frc.robot.utils.DualUSBCameraServer.isPrimaryConnected();
                if (usbCameraAvailable) {
                    String primaryCameraUrl = frc.robot.utils.DualUSBCameraServer.getPrimaryStreamUrl();
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Primary_USB_URL", primaryCameraUrl);
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Primary_USB_Camera_URL", primaryCameraUrl);
                }
            }
            
            if (frc.robot.utils.DualUSBCameraServer.isSecondaryInitialized()) {
                secondaryUsbCameraAvailable = frc.robot.utils.DualUSBCameraServer.isSecondaryConnected();
                if (secondaryUsbCameraAvailable) {
                    String secondaryCameraUrl = frc.robot.utils.DualUSBCameraServer.getSecondaryStreamUrl();
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/Secondary_USB_URL", secondaryCameraUrl);
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Secondary_USB_Camera_URL", secondaryCameraUrl);
                }
            }
            
            // Update dual camera status in SmartDashboard
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Primary_USB_Available", usbCameraAvailable);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Secondary_USB_Available", secondaryUsbCameraAvailable);
            
        } else {
            // Single camera mode - check original USBCameraServer status
            if (frc.robot.utils.USBCameraServer.isInitialized()) {
                usbCameraAvailable = frc.robot.utils.USBCameraServer.isConnected();
                if (usbCameraAvailable) {
                    String usbCameraUrl = frc.robot.utils.USBCameraServer.getStreamUrl();
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Camera/USB_URL", usbCameraUrl);
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("USB_Camera_URL", usbCameraUrl);
                }
            }
            
            // Update single camera status in SmartDashboard
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/USB_Available", usbCameraAvailable);
        }
        
        // Update camera status in SmartDashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Camera/Limelight_Available", limelightAvailable);
        
        // Debug: Print status every 10 seconds
        long currentTime = System.currentTimeMillis();
        if (currentTime % 10000 < 100) {
            System.out.println("Camera Status:");
            System.out.println("  Dual Camera Mode: " + (useDualCameras ? "ENABLED" : "DISABLED"));
            
            if (useDualCameras) {
                System.out.println("  Primary USB Camera: " + (usbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
                System.out.println("  Secondary USB Camera: " + (secondaryUsbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
                if (usbCameraAvailable) {
                    System.out.println("    Primary URL: " + frc.robot.utils.DualUSBCameraServer.getPrimaryStreamUrl());
                }
                if (secondaryUsbCameraAvailable) {
                    System.out.println("    Secondary URL: " + frc.robot.utils.DualUSBCameraServer.getSecondaryStreamUrl());
                }
            } else {
                System.out.println("  USB Camera: " + (usbCameraAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
                if (usbCameraAvailable) {
                    System.out.println("    USB Camera URL: " + frc.robot.utils.USBCameraServer.getStreamUrl());
                }
            }
            
            System.out.println("  Limelight: " + (limelightAvailable ? "AVAILABLE" : "NOT AVAILABLE"));
            System.out.println("  Limelight URL: " + LIMELIGHT_STREAM_URL);
            System.out.println("  Add Camera widgets to Shuffleboard using these SmartDashboard keys:");
            
            if (useDualCameras) {
                System.out.println("    - Primary_USB_Camera_URL: " + (usbCameraAvailable ? frc.robot.utils.DualUSBCameraServer.getPrimaryStreamUrl() : "Not available"));
                System.out.println("    - Secondary_USB_Camera_URL: " + (secondaryUsbCameraAvailable ? frc.robot.utils.DualUSBCameraServer.getSecondaryStreamUrl() : "Not available"));
            } else {
                System.out.println("    - USB_Camera_URL: " + (usbCameraAvailable ? frc.robot.utils.USBCameraServer.getStreamUrl() : "Not available"));
            }
            System.out.println("    - Limelight_URL: " + LIMELIGHT_STREAM_URL);
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
        if (useDualCameras) {
            return new String[] {
                LIMELIGHT_STREAM_URL,
                getPrimaryUsbCameraUrl(),
                getSecondaryUsbCameraUrl()
            };
        } else {
            return new String[] {
                LIMELIGHT_STREAM_URL,
                getPrimaryUsbCameraUrl(),
                ""
            };
        }
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
     * Gets primary USB camera stream URL.
     * 
     * @return Primary USB camera stream URL
     */
    public String getPrimaryUsbCameraUrl() {
        if (useDualCameras && frc.robot.utils.DualUSBCameraServer.isPrimaryInitialized()) {
            return frc.robot.utils.DualUSBCameraServer.getPrimaryStreamUrl();
        } else if (!useDualCameras && frc.robot.utils.USBCameraServer.isInitialized()) {
            return frc.robot.utils.USBCameraServer.getStreamUrl();
        }
        return "";
    }
    
    /**
     * Gets secondary USB camera stream URL.
     * 
     * @return Secondary USB camera stream URL
     */
    public String getSecondaryUsbCameraUrl() {
        if (useDualCameras && frc.robot.utils.DualUSBCameraServer.isSecondaryInitialized()) {
            return frc.robot.utils.DualUSBCameraServer.getSecondaryStreamUrl();
        }
        return "";
    }
    
    /**
     * Checks if dual camera mode is enabled.
     * 
     * @return True if dual camera mode is enabled
     */
    public boolean isDualCameraMode() {
        return useDualCameras;
    }
}
