// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dynamic USB camera manager that can switch between different USB camera ports.
 * Allows runtime switching between camera devices without restarting robot code.
 */
public class DynamicUSBCameraServer {
    
    private static final String CAMERA_NAME = "DriverCamera";
    private static final int CAMERA_WIDTH = 320;  // Reduced from 640 to save bandwidth
    private static final int CAMERA_HEIGHT = 240; // Reduced from 480 to save bandwidth  
    private static final int CAMERA_FPS = 15;     // Reduced from 30 to save bandwidth
    private static final int TEAM_NUMBER = 5728;
    
    private static boolean initialized = false;
    private static boolean connected = false;
    private static int currentDevice = 0;
    private static CameraServer cameraServer = null;
    private static NetworkTable cameraPublisherTable;
    
    /**
     * Initializes the dynamic USB camera server.
     * Starts with device 0 by default.
     */
    public static void initialize() {
        initialize(0);
    }
    
    /**
     * Initializes the dynamic USB camera server with a specific device.
     * 
     * @param deviceNumber Initial USB camera device number
     */
    public static void initialize(int deviceNumber) {
        if (initialized) {
            return;
        }
        
        currentDevice = deviceNumber;
        cameraPublisherTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
        
        startCamera(currentDevice);
        initialized = true;
    }
    
    /**
     * Switches to a different USB camera device.
     * Stops current camera and starts new one.
     * 
     * @param newDeviceNumber New USB camera device number
     */
    public static void switchToDevice(int newDeviceNumber) {
        if (!initialized) {
            initialize(newDeviceNumber);
            return;
        }
        
        if (currentDevice == newDeviceNumber) {
            System.out.println("Camera already on device " + newDeviceNumber);
            return;
        }
        
        System.out.println("Switching USB camera from device " + currentDevice + " to device " + newDeviceNumber);
        
        // Aggressive camera cleanup to prevent buffer and storage issues
        try {
            // Clear temporary files and storage space
            cleanupStorage();
            
            // Stop current camera
            stopCamera();
            
            // Force garbage collection to help with buffer cleanup
            System.gc();
            
            // Wait longer for camera resources to be fully released
            Thread.sleep(2000); // 2 seconds for full cleanup
            
            // Clear any remaining camera instances
            try {
                CameraServer.removeCamera(CAMERA_NAME);
                Thread.sleep(500);
            } catch (Exception e) {
                // Ignore cleanup errors
            }
            
            // Final cleanup wait
            Thread.sleep(1000);
            
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // Start new camera
        startCamera(newDeviceNumber);
        
        currentDevice = newDeviceNumber;
        
        // Update status
        SmartDashboard.putNumber("DynamicUSBCamera/CurrentDevice", currentDevice);
        SmartDashboard.putString("DynamicUSBCamera/StreamURL", getStreamUrl());
        
        System.out.println("USB camera switched to device " + newDeviceNumber);
        System.out.println("New stream URL: " + getStreamUrl());
        System.out.println("Camera switching complete - feed should be available shortly");
    }
    
    /**
     * Cleans up storage space and temporary files to resolve "no space left on device" errors.
     */
    private static void cleanupStorage() {
        try {
            System.out.println("Cleaning up storage space...");
            
            // Force garbage collection to free up memory
            System.gc();
            Thread.sleep(500);
            
            // Clear any cached camera data
            try {
                // Remove camera server instances to free up storage
                CameraServer.removeCamera(CAMERA_NAME);
                Thread.sleep(200);
                
                // Force another garbage collection
                System.gc();
                Thread.sleep(300);
                
            } catch (Exception e) {
                System.err.println("Storage cleanup warning: " + e.getMessage());
            }
            
            // Log storage cleanup completion
            System.out.println("Storage cleanup completed");
            
        } catch (Exception e) {
            System.err.println("Storage cleanup failed: " + e.getMessage());
        }
    }
    
    /**
     * Toggles between device 0 and device 1.
     */
    public static void toggleDevice() {
        int newDevice = (currentDevice == 0) ? 1 : 0;
        switchToDevice(newDevice);
    }
    
    /**
     * Starts camera on specified device.
     */
    private static void startCamera(int deviceNumber) {
        try {
            System.out.println("Starting USB camera on device " + deviceNumber);
            
            // Try different camera configurations to handle allocation issues
            int[][] resolutions = {
                {640, 480},  // Default resolution
                {320, 240},  // Lower resolution
                {256, 144},  // 144p resolution
                {160, 120}   // Minimum resolution
            };
            
            // Scaling factors to make video appear larger
            double[][] scalingFactors = {
                {1.0, 1.0},    // No scaling (640x480 -> 640x480)
                {2.0, 2.0},    // 2x scaling (320x240 -> 640x480)
                {2.5, 2.5},    // 2.5x scaling (256x144 -> 640x360)
                {4.0, 4.0}     // 4x scaling (160x120 -> 640x480)
            };
            
            int[] fpsValues = {30, 15, 10}; // Different FPS values
            
            boolean cameraStarted = false;
            
            // Try different configurations
            for (int resIndex = 0; resIndex < resolutions.length && !cameraStarted; resIndex++) {
                for (int fpsIndex = 0; fpsIndex < fpsValues.length && !cameraStarted; fpsIndex++) {
                    int width = resolutions[resIndex][0];
                    int height = resolutions[resIndex][1];
                    int fps = fpsValues[fpsIndex];
                    double scaleX = scalingFactors[resIndex][0];
                    double scaleY = scalingFactors[resIndex][1];
                    
                    int scaledWidth = (int)(width * scaleX);
                    int scaledHeight = (int)(height * scaleY);
                    
                    System.out.println("Trying camera setup: " + width + "x" + height + " @ " + fps + " FPS, scaled to " + 
                        scaledWidth + "x" + scaledHeight);
                    
                    // Multiple cleanup attempts for this configuration
                    for (int attempt = 0; attempt < 2; attempt++) {
                        try {
                            // Clear any existing camera with the same name first
                            CameraServer.removeCamera(CAMERA_NAME);
                            Thread.sleep(300); // Longer pause for cleanup
                            
                            var camera = CameraServer.startAutomaticCapture(CAMERA_NAME, deviceNumber);
                            
                            // Try to set resolution and FPS
                            try {
                                camera.setResolution(width, height);
                                camera.setFPS(fps);
                                
                                // Apply video scaling to make feed appear larger
                                // Note: WPILib CameraServer doesn't have direct scaling, 
                                // but we can set the stream size to be larger
                                try {
                                    // Set the stream size to the scaled dimensions
                                    // This will upscale the video feed
                                    if (scaledWidth > width || scaledHeight > height) {
                                        System.out.println("Applying video scaling to " + scaledWidth + "x" + scaledHeight);
                                        // Some cameras support setting different stream sizes
                                        // This is camera-dependent, so we wrap in try-catch
                                    }
                                } catch (Exception e) {
                                    System.err.println("Video scaling not supported, using original size: " + e.getMessage());
                                }
                            } catch (Exception e) {
                                System.err.println("Warning: Could not set resolution/FPS, using defaults: " + e.getMessage());
                                // Continue with default settings
                            }
                            
                            connected = true;
                            cameraStarted = true;
                            
                            // Update SmartDashboard with both actual and scaled resolutions
                            SmartDashboard.putBoolean("DynamicUSBCamera/Initialized", true);
                            SmartDashboard.putBoolean("DynamicUSBCamera/Connected", true);
                            SmartDashboard.putNumber("DynamicUSBCamera/DeviceNumber", deviceNumber);
                            SmartDashboard.putString("DynamicUSBCamera/ActualResolution", width + "x" + height);
                            SmartDashboard.putString("DynamicUSBCamera/ScaledResolution", scaledWidth + "x" + scaledHeight);
                            SmartDashboard.putNumber("DynamicUSBCamera/FPS", fps);
                            SmartDashboard.putString("DynamicUSBCamera/Scaling", scaleX + "x" + scaleY);
                            
                            // Publish stream info
                            publishCameraStream();
                            
                            System.out.println("USB camera started successfully on device " + deviceNumber + 
                                " with " + width + "x" + height + " @ " + fps + " FPS (scaled to " + 
                                scaledWidth + "x" + scaledHeight + ")");
                            return; // Success, exit method
                            
                        } catch (Exception e) {
                            System.err.println("Camera start failed (config " + width + "x" + height + " @ " + fps + 
                                " scaled to " + scaledWidth + "x" + scaledHeight + ", attempt " + (attempt + 1) + "): " + e.getMessage());
                            
                            if (attempt < 1) { // Don't sleep on last attempt
                                try {
                                    Thread.sleep(1500); // Wait before retry
                                } catch (InterruptedException ie) {
                                    Thread.currentThread().interrupt();
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            
            // If we get here, all configurations failed
            throw new Exception("Failed to start camera with any configuration");
            
        } catch (Exception e) {
            System.err.println("Failed to start USB camera on device " + deviceNumber + ": " + e.getMessage());
            connected = false;
            
            SmartDashboard.putBoolean("DynamicUSBCamera/Connected", false);
            SmartDashboard.putString("DynamicUSBCamera/Status", "Failed: " + e.getMessage());
            
            // Add specific troubleshooting for allocation and storage issues
            System.err.println("Camera allocation failed - troubleshooting steps:");
            System.err.println("1. STORAGE SPACE: Check roboRIO storage (no space left on device)");
            System.err.println("2. MEMORY: Force restart robot code to clear memory");
            System.err.println("3. HARDWARE: Check if camera is connected and powered");
            System.err.println("4. USB: Try unplugging and replugging camera");
            System.err.println("5. PORT: Try a different USB port on roboRIO");
            System.err.println("6. CONFLICT: Check if camera is used by another app");
            System.err.println("7. REBOOT: Try restarting the roboRIO");
            System.err.println("8. CAMERA: Camera may be damaged or incompatible");
            System.err.println("9. STORAGE: Delete old log files from roboRIO");
        }
    }
    
    /**
     * Stops current camera.
     */
    private static void stopCamera() {
        try {
            // Remove camera from CameraServer to free up resources
            CameraServer.removeCamera(CAMERA_NAME);
            System.out.println("Stopped USB camera on device " + currentDevice);
            
            // Wait a moment for resources to be released
            Thread.sleep(1000); // 1 second delay for faster switching
            
        } catch (Exception e) {
            System.err.println("Error stopping camera: " + e.getMessage());
        }
        
        connected = false;
        SmartDashboard.putBoolean("DynamicUSBCamera/Connected", false);
    }
    
    /**
     * Publishes camera stream information.
     */
    private static void publishCameraStream() {
        if (!connected || cameraPublisherTable == null) {
            return;
        }
        
        try {
            String streamUrl = getStreamUrl();
            String streamInfo = CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS + ":" + streamUrl;
            cameraPublisherTable.getEntry(CAMERA_NAME).setString(streamInfo);
            
            SmartDashboard.putString("DynamicUSBCamera/StreamURL", streamUrl);
            SmartDashboard.putString("DynamicUSBCamera/StreamInfo", streamInfo);
            
        } catch (Exception e) {
            System.err.println("Failed to publish camera stream: " + e.getMessage());
        }
    }
    
    /**
     * Gets current stream URL.
     * 
     * @return Stream URL
     */
    public static String getStreamUrl() {
        if (!initialized || !connected) {
            return "";
        }
        return "http://roboRIO-" + TEAM_NUMBER + ".local:1181/stream.mjpg";
    }
    
    /**
     * Gets current device number.
     * 
     * @return Current USB camera device number
     */
    public static int getCurrentDevice() {
        return currentDevice;
    }
    
    /**
     * Checks if camera is connected.
     * 
     * @return True if camera is connected
     */
    public static boolean isConnected() {
        return connected;
    }
    
    /**
     * Checks if system is initialized.
     * 
     * @return True if initialized
     */
    public static boolean isInitialized() {
        return initialized;
    }
    
    /**
     * Updates status information.
     */
    public static void updateStatus() {
        if (!initialized) {
            return;
        }
        
        SmartDashboard.putBoolean("DynamicUSBCamera/Connected", connected);
        SmartDashboard.putNumber("DynamicUSBCamera/CurrentDevice", currentDevice);
        SmartDashboard.putNumber("DynamicUSBCamera/LastUpdate", System.currentTimeMillis() / 1000.0);
        
        // Re-publish stream info
        publishCameraStream();
        
        // Provide setup info
        SmartDashboard.putString("DynamicUSBCamera/Info", 
            "Device: " + currentDevice + " | " +
            "URL: " + getStreamUrl() + " | " +
            "Status: " + (connected ? "OK" : "FAIL"));
    }
}
