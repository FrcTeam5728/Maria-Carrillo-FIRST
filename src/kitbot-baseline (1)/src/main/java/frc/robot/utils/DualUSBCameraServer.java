// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class to manage multiple USB webcam CameraServer integration.
 * Supports dual camera feeds for driver view and secondary camera in Shuffleboard.
 * Uses NetworkTables approach for maximum compatibility.
 */
public class DualUSBCameraServer {
    
    // Camera configuration constants
    private static final String PRIMARY_CAMERA_NAME = "DriverCamera";
    private static final String SECONDARY_CAMERA_NAME = "SecondaryCamera";
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final int CAMERA_FPS = 30;
    private static final int PRIMARY_DEVICE = 0;
    private static final int SECONDARY_DEVICE = 1;
    
    // Initialization state
    private static boolean primaryInitialized = false;
    private static boolean secondaryInitialized = false;
    private static boolean primaryConnected = false;
    private static boolean secondaryConnected = false;
    
    // NetworkTables for camera publishing
    private static NetworkTable cameraPublisherTable;
    
    /**
     * Initializes both USB cameras with default device numbers.
     * Primary camera: device 0, Secondary camera: device 1
     */
    public static void initialize() {
        initialize(PRIMARY_DEVICE, SECONDARY_DEVICE);
    }
    
    /**
     * Initializes both USB cameras with specified device numbers.
     * 
     * @param primaryDevice Device number for primary camera (usually 0)
     * @param secondaryDevice Device number for secondary camera (usually 1)
     */
    public static void initialize(int primaryDevice, int secondaryDevice) {
        // Set up NetworkTables for camera publishing
        cameraPublisherTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
        
        // Initialize primary camera
        initializePrimaryCamera(primaryDevice);
        
        // Initialize secondary camera
        initializeSecondaryCamera(secondaryDevice);
        
        // Publish overall status
        publishDualCameraStatus();
        
        System.out.println("=== DUAL USB CAMERA SYSTEM ===");
        System.out.println("Primary Camera: " + (primaryConnected ? "CONNECTED" : "NOT CONNECTED"));
        System.out.println("Secondary Camera: " + (secondaryConnected ? "CONNECTED" : "NOT CONNECTED"));
        System.out.println("Primary URL: " + getPrimaryStreamUrl());
        System.out.println("Secondary URL: " + getSecondaryStreamUrl());
        System.out.println("===============================");
    }
    
    /**
     * Initializes the primary USB camera.
     */
    private static void initializePrimaryCamera(int deviceNumber) {
        try {
            System.out.println("Starting Primary USB Camera Server...");
            
            var camera = CameraServer.startAutomaticCapture(PRIMARY_CAMERA_NAME, deviceNumber);
            camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            camera.setFPS(CAMERA_FPS);
            
            primaryInitialized = true;
            primaryConnected = true;
            
            // Update SmartDashboard with primary camera status
            SmartDashboard.putBoolean("DualUSBCamera/Primary/Initialized", true);
            SmartDashboard.putBoolean("DualUSBCamera/Primary/Connected", true);
            SmartDashboard.putString("DualUSBCamera/Primary/CameraName", PRIMARY_CAMERA_NAME);
            SmartDashboard.putNumber("DualUSBCamera/Primary/DeviceNumber", deviceNumber);
            
            // Publish primary camera stream
            publishPrimaryCameraStream();
            
            System.out.println("Primary camera initialized successfully");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize primary USB camera: " + e.getMessage());
            primaryInitialized = false;
            primaryConnected = false;
            
            SmartDashboard.putBoolean("DualUSBCamera/Primary/Initialized", false);
            SmartDashboard.putBoolean("DualUSBCamera/Primary/Connected", false);
            SmartDashboard.putString("DualUSBCamera/Primary/Status", "Failed: " + e.getMessage());
        }
    }
    
    /**
     * Initializes the secondary USB camera.
     */
    private static void initializeSecondaryCamera(int deviceNumber) {
        try {
            System.out.println("Starting Secondary USB Camera Server...");
            
            var camera = CameraServer.startAutomaticCapture(SECONDARY_CAMERA_NAME, deviceNumber);
            camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            camera.setFPS(CAMERA_FPS);
            
            secondaryInitialized = true;
            secondaryConnected = true;
            
            // Update SmartDashboard with secondary camera status
            SmartDashboard.putBoolean("DualUSBCamera/Secondary/Initialized", true);
            SmartDashboard.putBoolean("DualUSBCamera/Secondary/Connected", true);
            SmartDashboard.putString("DualUSBCamera/Secondary/CameraName", SECONDARY_CAMERA_NAME);
            SmartDashboard.putNumber("DualUSBCamera/Secondary/DeviceNumber", deviceNumber);
            
            // Publish secondary camera stream
            publishSecondaryCameraStream();
            
            System.out.println("Secondary camera initialized successfully");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize secondary USB camera: " + e.getMessage());
            secondaryInitialized = false;
            secondaryConnected = false;
            
            SmartDashboard.putBoolean("DualUSBCamera/Secondary/Initialized", false);
            SmartDashboard.putBoolean("DualUSBCamera/Secondary/Connected", false);
            SmartDashboard.putString("DualUSBCamera/Secondary/Status", "Failed: " + e.getMessage());
        }
    }
    
    /**
     * Publishes the primary camera stream information to NetworkTables.
     */
    private static void publishPrimaryCameraStream() {
        if (!primaryInitialized || cameraPublisherTable == null) {
            return;
        }
        
        try {
            String streamUrl = getPrimaryStreamUrl();
            String streamInfo = PRIMARY_CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS + ":" + streamUrl;
            cameraPublisherTable.getEntry(PRIMARY_CAMERA_NAME).setString(streamInfo);
            
            SmartDashboard.putString("DualUSBCamera/Primary/StreamURL", streamUrl);
            SmartDashboard.putString("DualUSBCamera/Primary/StreamInfo", streamInfo);
            
        } catch (Exception e) {
            System.err.println("Failed to publish primary camera stream: " + e.getMessage());
        }
    }
    
    /**
     * Publishes the secondary camera stream information to NetworkTables.
     */
    private static void publishSecondaryCameraStream() {
        if (!secondaryInitialized || cameraPublisherTable == null) {
            return;
        }
        
        try {
            String streamUrl = getSecondaryStreamUrl();
            String streamInfo = SECONDARY_CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS + ":" + streamUrl;
            cameraPublisherTable.getEntry(SECONDARY_CAMERA_NAME).setString(streamInfo);
            
            SmartDashboard.putString("DualUSBCamera/Secondary/StreamURL", streamUrl);
            SmartDashboard.putString("DualUSBCamera/Secondary/StreamInfo", streamInfo);
            
        } catch (Exception e) {
            System.err.println("Failed to publish secondary camera stream: " + e.getMessage());
        }
    }
    
    /**
     * Publishes overall dual camera system status.
     */
    private static void publishDualCameraStatus() {
        try {
            SmartDashboard.putBoolean("DualUSBCamera/System/Initialized", primaryInitialized || secondaryInitialized);
            SmartDashboard.putNumber("DualUSBCamera/System/ConnectedCameras", 
                                   (primaryConnected ? 1 : 0) + (secondaryConnected ? 1 : 0));
            SmartDashboard.putString("DualUSBCamera/System/Status", 
                                   "Primary: " + (primaryConnected ? "OK" : "FAIL") + 
                                   " | Secondary: " + (secondaryConnected ? "OK" : "FAIL"));
            
        } catch (Exception e) {
            System.err.println("Failed to publish dual camera status: " + e.getMessage());
        }
    }
    
    /**
     * Gets the stream URL for the primary USB camera.
     * 
     * @return Stream URL for primary camera
     */
    public static String getPrimaryStreamUrl() {
        if (!primaryInitialized) {
            return "";
        }
        return "http://roboRIO-5728.local:1181/stream.mjpg";
    }
    
    /**
     * Gets the stream URL for the secondary USB camera.
     * 
     * @return Stream URL for secondary camera
     */
    public static String getSecondaryStreamUrl() {
        if (!secondaryInitialized) {
            return "";
        }
        return "http://roboRIO-5728.local:1182/stream.mjpg";
    }
    
    /**
     * Gets the primary camera name.
     * 
     * @return Primary camera name
     */
    public static String getPrimaryCameraName() {
        return PRIMARY_CAMERA_NAME;
    }
    
    /**
     * Gets the secondary camera name.
     * 
     * @return Secondary camera name
     */
    public static String getSecondaryCameraName() {
        return SECONDARY_CAMERA_NAME;
    }
    
    /**
     * Checks if the primary camera is initialized.
     * 
     * @return True if primary camera is initialized
     */
    public static boolean isPrimaryInitialized() {
        return primaryInitialized;
    }
    
    /**
     * Checks if the secondary camera is initialized.
     * 
     * @return True if secondary camera is initialized
     */
    public static boolean isSecondaryInitialized() {
        return secondaryInitialized;
    }
    
    /**
     * Checks if the primary camera is connected.
     * 
     * @return True if primary camera is connected
     */
    public static boolean isPrimaryConnected() {
        return primaryConnected;
    }
    
    /**
     * Checks if the secondary camera is connected.
     * 
     * @return True if secondary camera is connected
     */
    public static boolean isSecondaryConnected() {
        return secondaryConnected;
    }
    
    /**
     * Updates the status information for debugging.
     * This should be called periodically.
     */
    public static void updateStatus() {
        try {
            // Update connection status
            SmartDashboard.putBoolean("DualUSBCamera/Primary/Connected", primaryConnected);
            SmartDashboard.putBoolean("DualUSBCamera/Secondary/Connected", secondaryConnected);
            SmartDashboard.putNumber("DualUSBCamera/System/LastUpdate", 
                                   System.currentTimeMillis() / 1000.0);
            
            // Re-publish camera streams
            publishPrimaryCameraStream();
            publishSecondaryCameraStream();
            publishDualCameraStatus();
            
            // Provide setup info for Shuffleboard
            SmartDashboard.putString("DualUSBCamera/ShuffleboardSetup", 
                "PRIMARY CAMERA:\n" +
                "1. Open Shuffleboard\n" +
                "2. Click '+' to add widget\n" +
                "3. Select 'Camera Server'\n" +
                "4. Choose '" + PRIMARY_CAMERA_NAME + "'\n\n" +
                "SECONDARY CAMERA:\n" +
                "1. Click '+' to add widget\n" +
                "2. Select 'Camera Server'\n" +
                "3. Choose '" + SECONDARY_CAMERA_NAME + "'");
            
        } catch (Exception e) {
            System.err.println("Error updating dual USB camera status: " + e.getMessage());
        }
    }
    
    /**
     * Gets setup instructions for both cameras.
     * 
     * @return Setup instructions for dual camera system
     */
    public static String getSetupInstructions() {
        StringBuilder info = new StringBuilder();
        info.append("=== DUAL USB CAMERA SETUP ===\n");
        info.append("Primary Camera: ").append(PRIMARY_CAMERA_NAME).append("\n");
        info.append("Secondary Camera: ").append(SECONDARY_CAMERA_NAME).append("\n");
        info.append("Resolution: ").append(CAMERA_WIDTH).append("x").append(CAMERA_HEIGHT).append("\n");
        info.append("FPS: ").append(CAMERA_FPS).append("\n");
        info.append("Team Number: 5728\n\n");
        
        info.append("PRIMARY CAMERA URL: ").append(getPrimaryStreamUrl()).append("\n");
        info.append("SECONDARY CAMERA URL: ").append(getSecondaryStreamUrl()).append("\n\n");
        
        info.append("SHUFFLEBOARD SETUP:\n");
        info.append("PRIMARY CAMERA:\n");
        info.append("1. Open Shuffleboard\n");
        info.append("2. Click '+' to add widget\n");
        info.append("3. Select 'Camera Server'\n");
        info.append("4. Choose '").append(PRIMARY_CAMERA_NAME).append("'\n\n");
        
        info.append("SECONDARY CAMERA:\n");
        info.append("1. Click '+' to add widget\n");
        info.append("2. Select 'Camera Server'\n");
        info.append("3. Choose '").append(SECONDARY_CAMERA_NAME).append("'\n\n");
        
        info.append("TROUBLESHOOTING:\n");
        info.append("• Check USB camera connections to roboRIO\n");
        info.append("• Try different device numbers (0, 1, 2, 3)\n");
        info.append("• Verify cameras work on computer first\n");
        info.append("• Check camera permissions on roboRIO\n");
        info.append("• Restart robot code if needed\n");
        info.append("================================");
        return info.toString();
    }
    
    /**
     * Prints setup instructions to console.
     */
    public static void printSetupInstructions() {
        System.out.println(getSetupInstructions());
    }
}
