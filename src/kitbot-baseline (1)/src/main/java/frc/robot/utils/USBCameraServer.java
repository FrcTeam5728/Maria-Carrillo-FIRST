// Copyright (c) FIRST and other WPilb contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.config.CameraConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class to manage USB webcam CameraServer integration.
 * Provides a dedicated camera feed for driver view in Shuffleboard.
 * Uses NetworkTables approach for maximum compatibility.
 */
public class USBCameraServer {
    
    private static final String CAMERA_NAME = CameraConfig.PRIMARY_CAMERA_NAME;
    private static final int CAMERA_WIDTH = CameraConfig.CAMERA_WIDTH;
    private static final int CAMERA_HEIGHT = CameraConfig.CAMERA_HEIGHT;
    private static final int CAMERA_FPS = CameraConfig.CAMERA_FPS;
    private static final int DEFAULT_DEVICE = CameraConfig.PRIMARY_USB_CAMERA_DEVICE;
    
    private static boolean isInitialized = false;
    private static boolean isConnected = false;
    private static int deviceNumber = DEFAULT_DEVICE;
    
    private static NetworkTable cameraPublisherTable;
    
    /**
     * Initializes the USB CameraServer.
     * Automatically finds and configures the first available USB camera.
     */
    public static void initialize() {
        initialize(DEFAULT_DEVICE);
    }
    
    /**
     * Initializes with specific camera device number.
     * Use this if you have multiple USB cameras.
     * 
     * @param deviceNumber USB camera device number (0, 1, 2, etc.)
     */
    public static void initialize(int deviceNumber) {
        if (isInitialized) {
            return;
        }
        
        try {
            // Store device number
            USBCameraServer.deviceNumber = deviceNumber;
            
            // Set up NetworkTables for camera publishing
            cameraPublisherTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
            
            // Start the USB camera server with WPILib CameraServer
            System.out.println("Starting USB Camera Server...");
            
            // Start automatic capture using WPILib CameraServer with specific device
            // This will automatically detect and start streaming from the specified USB camera
            var camera = CameraServer.startAutomaticCapture(deviceNumber);
            camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            camera.setFPS(CAMERA_FPS);
            
            isInitialized = true;
            isConnected = true;
            
            // Update SmartDashboard with status
            SmartDashboard.putBoolean("USBCameraServer/Initialized", true);
            SmartDashboard.putBoolean("USBCameraServer/Connected", true);
            SmartDashboard.putString("USBCameraServer/CameraName", CAMERA_NAME);
            SmartDashboard.putNumber("USBCameraServer/DeviceNumber", deviceNumber);
            SmartDashboard.putNumber("USBCameraServer/Width", CAMERA_WIDTH);
            SmartDashboard.putNumber("USBCameraServer/Height", CAMERA_HEIGHT);
            SmartDashboard.putNumber("USBCameraServer/FPS", CAMERA_FPS);
            
            // Publish camera stream info
            publishCameraStream();
            
            System.out.println("=== USB CAMERA SERVER INITIALIZED ===");
            System.out.println("Camera Name: " + CAMERA_NAME);
            System.out.println("Device Number: " + deviceNumber);
            System.out.println("Resolution: " + CAMERA_WIDTH + "x" + CAMERA_HEIGHT);
            System.out.println("FPS: " + CAMERA_FPS);
            System.out.println("Team Number: 5728");
            System.out.println("Stream URL: " + getStreamUrl());
            System.out.println("Status: Connected and streaming to Shuffleboard");
            System.out.println("=====================================");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize USB CameraServer with device " + deviceNumber + ": " + e.getMessage());
            isInitialized = false;
            isConnected = false;
            
            SmartDashboard.putBoolean("USBCameraServer/Initialized", false);
            SmartDashboard.putBoolean("USBCameraServer/Connected", false);
            SmartDashboard.putString("USBCameraServer/Status", "Failed: " + e.getMessage());
        }
    }
    
    /**
     * Publishes the camera stream information to NetworkTables.
     * This makes the camera available to Shuffleboard.
     */
    private static void publishCameraStream() {
        if (!isInitialized || cameraPublisherTable == null) {
            return;
        }
        
        try {
            // Publish camera stream information
            // The actual camera feed is handled by WPILib's CameraServer
            // Stream URL format: http://roboRIO-TEAM.local:1181/stream.mjpg
            
            String streamUrl = getStreamUrl();
            String streamInfo = CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS + ":" + streamUrl;
            cameraPublisherTable.getEntry(CAMERA_NAME).setString(streamInfo);
            
            SmartDashboard.putString("USBCameraServer/StreamInfo", streamInfo);
            SmartDashboard.putString("USBCameraServer/StreamURL", streamUrl);
            
            // Note: Team number 5728 is configured
            SmartDashboard.putString("USBCameraServer/Note", "Team 5728 camera server active");
            
        } catch (Exception e) {
            System.err.println("Failed to publish USB camera stream: " + e.getMessage());
        }
    }
    
    /**
     * Gets the camera name for Shuffleboard configuration.
     * 
     * @return Camera name
     */
    public static String getCameraName() {
        return CAMERA_NAME;
    }
    
    /**
     * Gets the stream URL for the USB camera.
     * 
     * @return Stream URL with team number 5728
     */
    public static String getStreamUrl() {
        if (!isInitialized) {
            return "";
        }
        
        // Use team number 5728
        return "http://roboRIO-" + CameraConfig.TEAM_NUMBER + ".local:" + CameraConfig.PRIMARY_CAMERA_STREAM_PORT + "/stream.mjpg";
    }
    
    /**
     * Gets the stream URL for the USB camera with a specific team number.
     * 
     * @param teamNumber The team number to use in the URL
     * @return Stream URL with specified team number
     */
    public static String getStreamUrl(int teamNumber) {
        if (!isInitialized) {
            return "";
        }
        return "http://roboRIO-" + teamNumber + ".local:" + CameraConfig.PRIMARY_CAMERA_STREAM_PORT + "/stream.mjpg";
    }
    
    /**
     * Gets the device number being used.
     * 
     * @return Device number
     */
    public static int getDeviceNumber() {
        return deviceNumber;
    }
    
    /**
     * Checks if the USB CameraServer is initialized.
     * 
     * @return True if initialized
     */
    public static boolean isInitialized() {
        return isInitialized;
    }
    
    /**
     * Checks if the USB camera is connected.
     * 
     * @return True if connected
     */
    public static boolean isConnected() {
        return isConnected;
    }
    
    /**
     * Updates the status information for debugging.
     * This should be called periodically.
     */
    public static void updateStatus() {
        if (!isInitialized) {
            return;
        }
        
        try {
            // Update connection status
            SmartDashboard.putBoolean("USBCameraServer/Connected", isConnected);
            SmartDashboard.putNumber("USBCameraServer/LastUpdate", 
                                   System.currentTimeMillis() / 1000.0);
            
            // Re-publish camera stream info
            publishCameraStream();
            
            // Provide setup info for Shuffleboard
            SmartDashboard.putString("USBCameraServer/ShuffleboardSetup", 
                "1. Open Shuffleboard" +
                "\\n2. Click '+' to add widget" +
                "\\n3. Select 'Camera Server'" +
                "\\n4. Choose '" + CAMERA_NAME + "'" +
                "\\n5. Camera feed should appear");
            
            // Provide troubleshooting info
            SmartDashboard.putString("USBCameraServer/Troubleshooting", 
                "1. Check USB camera connection" +
                "\\n2. Verify camera device number (try 0, 1, 2)" +
                "\\n3. Check camera permissions on roboRIO" +
                "\\n4. Restart robot code if needed" +
                "\\n5. Check if camera is recognized by system");
            
        } catch (Exception e) {
            System.err.println("Error updating USB CameraServer status: " + e.getMessage());
        }
    }
    
    /**
     * Gets setup instructions for Shuffleboard.
     * 
     * @return Setup instructions
     */
    public static String getSetupInstructions() {
        StringBuilder info = new StringBuilder();
        info.append("=== USB CAMERA SHUFFLEBOARD SETUP ===\n");
        info.append("Camera Name: ").append(CAMERA_NAME).append("\n");
        info.append("Device Number: ").append(deviceNumber).append("\n");
        info.append("Resolution: ").append(CAMERA_WIDTH).append("x").append(CAMERA_HEIGHT).append("\n");
        info.append("FPS: ").append(CAMERA_FPS).append("\n");
        info.append("Team Number: 5728\n");
        info.append("Stream URL: ").append(getStreamUrl()).append("\n\n");
        info.append("SETUP STEPS:\n");
        info.append("1. Open Shuffleboard\n");
        info.append("2. Click '+' to add widget\n");
        info.append("3. Select 'Camera Server'\n");
        info.append("4. Choose '").append(CAMERA_NAME).append("'\n");
        info.append("5. Camera feed should appear automatically\n\n");
        info.append("ALTERNATIVE SETUP:\n");
        info.append("1. In Shuffleboard, right-click\n");
        info.append("2. Select 'Add Camera'\n");
        info.append("3. USB camera should auto-detect as '").append(CAMERA_NAME).append("'\n\n");
        info.append("DIRECT URL ACCESS:\n");
        info.append("• Open browser: ").append(getStreamUrl()).append("\n");
        info.append("• Works on any device on the same network as roboRIO\n\n");
        info.append("TROUBLESHOOTING:\n");
        info.append("• Check USB camera connection to roboRIO\n");
        info.append("• Try different device numbers (0, 1, 2)\n");
        info.append("• Check camera permissions on roboRIO\n");
        info.append("• Verify camera works on computer first\n");
        info.append("• Restart robot code if needed\n");
        info.append("• Check roboRIO web console for camera detection\n");
        info.append("• Ensure team number 5728 is correctly configured\n");
        info.append("==========================================");
        return info.toString();
    }
    
    /**
     * Prints setup instructions to console.
     */
    public static void printSetupInstructions() {
        System.out.println(getSetupInstructions());
    }
    
    /**
     * Tries to initialize with a different device number.
     * Useful for troubleshooting multiple USB cameras.
     * 
     * @param newDeviceNumber New device number to try
     */
    public static void tryDeviceNumber(int newDeviceNumber) {
        System.out.println("Trying USB camera device number: " + newDeviceNumber);
        
        // Reset initialization state
        isInitialized = false;
        isConnected = false;
        
        // Try with new device number
        initialize(newDeviceNumber);
    }
}
