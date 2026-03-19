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
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final int CAMERA_FPS = 30;
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
        
        // Aggressive camera cleanup to prevent buffer issues
        try {
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
            
            // Multiple cleanup attempts to ensure buffer is clear
            for (int i = 0; i < 3; i++) {
                try {
                    // Clear any existing camera with the same name first
                    CameraServer.removeCamera(CAMERA_NAME);
                    Thread.sleep(200); // Brief pause for cleanup
                    
                    var camera = CameraServer.startAutomaticCapture(CAMERA_NAME, deviceNumber);
                    camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
                    camera.setFPS(CAMERA_FPS);
                    
                    connected = true;
                    
                    // Update SmartDashboard
                    SmartDashboard.putBoolean("DynamicUSBCamera/Initialized", true);
                    SmartDashboard.putBoolean("DynamicUSBCamera/Connected", true);
                    SmartDashboard.putNumber("DynamicUSBCamera/DeviceNumber", deviceNumber);
                    
                    // Publish stream info
                    publishCameraStream();
                    
                    System.out.println("USB camera started successfully on device " + deviceNumber + " (attempt " + (i + 1) + ")");
                    return; // Success, exit method
                    
                } catch (Exception e) {
                    System.err.println("Camera start attempt " + (i + 1) + " failed: " + e.getMessage());
                    
                    if (i < 2) { // Don't sleep on last attempt
                        try {
                            Thread.sleep(1000); // Wait before retry
                        } catch (InterruptedException ie) {
                            Thread.currentThread().interrupt();
                            break;
                        }
                    }
                }
            }
            
            // If we get here, all attempts failed
            throw new Exception("Failed to start camera after 3 attempts");
            
        } catch (Exception e) {
            System.err.println("Failed to start USB camera on device " + deviceNumber + ": " + e.getMessage());
            connected = false;
            
            SmartDashboard.putBoolean("DynamicUSBCamera/Connected", false);
            SmartDashboard.putString("DynamicUSBCamera/Status", "Failed: " + e.getMessage());
            
            // Add additional debugging info
            System.err.println("Camera start failed - possible causes:");
            System.err.println("- Camera device not found or disconnected");
            System.err.println("- Camera buffer full (try unplugging and replugging camera)");
            System.err.println("- Camera already in use by another application");
            System.err.println("- Insufficient USB bandwidth or power");
            System.err.println("- USB port issues or cable problems");
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
            Thread.sleep(100);
            
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
