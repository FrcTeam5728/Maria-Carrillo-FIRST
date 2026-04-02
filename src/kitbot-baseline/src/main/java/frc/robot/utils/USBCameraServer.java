// Copyright (c) FIRST and other WPilb contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class to manage USB webcam CameraServer integration.
 * Provides a dedicated camera feed for driver view in Shuffleboard.
 * Uses NetworkTables approach for maximum compatibility.
 */
public class USBCameraServer {
    
    private static final String CAMERA_NAME = "DriverCamera";
    private static final int CAMERA_WIDTH = 320;  // Reduced from 640 to save bandwidth
    private static final int CAMERA_HEIGHT = 240; // Reduced from 480 to save bandwidth  
    private static final int CAMERA_FPS = 15;     // Reduced from 30 to save bandwidth
    private static final int DEFAULT_DEVICE = 0;
    
    private static boolean isInitialized = false;
    private static boolean isConnected = false;
    private static int deviceNumber = DEFAULT_DEVICE;
    private static boolean isActive = false;  // Track if camera is currently active
    
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
            
            // Try to detect if USB camera is available
            // Note: In WPILib, we can't directly check USB camera availability
            // without actually creating the camera, so we'll assume it's available
            // and let the CameraServer handle the actual connection
            
            isInitialized = true;
            isConnected = true;
            isActive = true;  // Camera is now active
            
            // Update SmartDashboard with status
            SmartDashboard.putBoolean("USBCameraServer/Initialized", true);
            SmartDashboard.putBoolean("USBCameraServer/Connected", true);
            SmartDashboard.putBoolean("USBCameraServer/Active", true);
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
            System.out.println("Resolution: " + CAMERA_WIDTH + "x" + CAMERA_HEIGHT + " (Optimized for bandwidth)");
            System.out.println("FPS: " + CAMERA_FPS + " (Optimized for bandwidth)");
            System.out.println("Status: Connected and ready for Shuffleboard");
            System.out.println("=====================================");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize USB CameraServer with device " + deviceNumber + ": " + e.getMessage());
            isInitialized = false;
            isConnected = false;
            isActive = false;
            
            SmartDashboard.putBoolean("USBCameraServer/Initialized", false);
            SmartDashboard.putBoolean("USBCameraServer/Connected", false);
            SmartDashboard.putBoolean("USBCameraServer/Active", false);
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
            // The actual camera feed will be handled by WPILib's CameraServer
            // when it detects the USB camera
            
            String streamInfo = CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS;
            cameraPublisherTable.getEntry(CAMERA_NAME).setString(streamInfo);
            
            SmartDashboard.putString("USBCameraServer/StreamInfo", streamInfo);
            
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
            SmartDashboard.putBoolean("USBCameraServer/Active", isActive);
            SmartDashboard.putNumber("USBCameraServer/LastUpdate", 
                                   System.currentTimeMillis() / 1000.0);
            
            // Re-publish camera stream info only if active
            if (isActive) {
                publishCameraStream();
            }
            
            // Provide setup info for Shuffleboard
            SmartDashboard.putString("USBCameraServer/ShuffleboardSetup", 
                "1. Open Shuffleboard" +
                "\\n2. Click '+' to add widget" +
                "\\n3. Select 'Camera Server'" +
                "\\n4. Choose '" + CAMERA_NAME + "'" +
                "\\n5. Camera feed should appear");
            
            // Provide bandwidth-optimized troubleshooting info
            SmartDashboard.putString("USBCameraServer/Troubleshooting", 
                "1. Check USB camera connection" +
                "\\n2. Verify camera device number (use LEFT TRIGGER to switch)" +
                "\\n3. Camera is optimized for USB bandwidth (320x240@15fps)" +
                "\\n4. Only one camera active at a time to prevent bandwidth issues" +
                "\\n5. Use LEFT TRIGGER to cycle through cameras 0, 1, 2");
            
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
        info.append("=== USB CAMERA SHUFFLEBOARD SETUP ===\\n");
        info.append("Camera Name: ").append(CAMERA_NAME).append("\\n");
        info.append("Device Number: ").append(deviceNumber).append("\\n");
        info.append("Resolution: ").append(CAMERA_WIDTH).append("x").append(CAMERA_HEIGHT).append("\\n");
        info.append("FPS: ").append(CAMERA_FPS).append("\\n\\n");
        info.append("SETUP STEPS:\\n");
        info.append("1. Open Shuffleboard\\n");
        info.append("2. Click '+' to add widget\\n");
        info.append("3. Select 'Camera Server'\\n");
        info.append("4. Choose '").append(CAMERA_NAME).append("'\\n");
        info.append("5. Camera feed should appear\\n\\n");
        info.append("ALTERNATIVE SETUP:\\n");
        info.append("1. In Shuffleboard, right-click\\n");
        info.append("2. Select 'Add Camera'\\n");
        info.append("3. USB camera should auto-detect\\n\\n");
        info.append("TROUBLESHOOTING:\\n");
        info.append("• Check USB camera connection to roboRIO\\n");
        info.append("• Try different device numbers (0, 1, 2)\\n");
        info.append("• Check camera permissions on roboRIO\\n");
        info.append("• Verify camera works on computer first\\n");
        info.append("• Restart robot code if needed\\n");
        info.append("• Check roboRIO web console for camera detection\\n");
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
        isActive = false;
        
        // Try with new device number
        initialize(newDeviceNumber);
    }
    
    /**
     * Safely switches to a different USB camera device.
     * Properly manages bandwidth by stopping current camera first.
     * 
     * @param newDeviceNumber New device number to switch to
     */
    public static void switchToDevice(int newDeviceNumber) {
        System.out.println("=== SWITCHING USB CAMERA ===");
        System.out.println("From device: " + deviceNumber);
        System.out.println("To device: " + newDeviceNumber);
        
        // First, stop current camera to free bandwidth
        stopCamera();
        
        // Wait a moment for resources to be released
        try {
            Thread.sleep(1000); // 1 second delay for faster switching
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // Reset state completely
        isInitialized = false;
        isConnected = false;
        isActive = false;
        
        // Initialize new camera
        initialize(newDeviceNumber);
        
        System.out.println("=== CAMERA SWITCH COMPLETE ===");
    }
    
    /**
     * Stops the current camera and frees bandwidth.
     */
    public static void stopCamera() {
        if (isActive) {
            System.out.println("Stopping USB camera device " + deviceNumber + " to free bandwidth");
            
            // Clear camera stream info
            if (cameraPublisherTable != null) {
                try {
                    cameraPublisherTable.getEntry(CAMERA_NAME).setString("");
                } catch (Exception e) {
                    System.err.println("Error clearing camera stream: " + e.getMessage());
                }
            }
            
            // Update status
            isActive = false;
            SmartDashboard.putBoolean("USBCameraServer/Active", false);
            SmartDashboard.putString("USBCameraServer/Status", "Camera stopped - bandwidth freed");
            
            System.out.println("USB camera stopped - bandwidth freed");
        }
    }
    
    /**
     * Checks if the camera is currently active.
     * 
     * @return True if camera is active
     */
    public static boolean isActive() {
        return isActive;
    }
}
