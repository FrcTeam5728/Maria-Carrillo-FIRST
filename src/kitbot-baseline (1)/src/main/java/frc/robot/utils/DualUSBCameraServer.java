// Copyright (c) FIRST and other WPILib contributors.
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
 * Dual USB Camera Server utility class.
 * Manages two USB cameras simultaneously for complete driver visibility.
 * Provides both camera feeds to Shuffleboard for side-by-side viewing.
 */
public class DualUSBCameraServer {
    
    // Primary camera configuration
    private static final String PRIMARY_CAMERA_NAME = CameraConfig.PRIMARY_CAMERA_NAME;
    private static final int PRIMARY_DEVICE = CameraConfig.PRIMARY_USB_CAMERA_DEVICE;
    private static final int PRIMARY_PORT = CameraConfig.PRIMARY_CAMERA_STREAM_PORT;
    
    // Secondary camera configuration  
    private static final String SECONDARY_CAMERA_NAME = CameraConfig.SECONDARY_CAMERA_NAME;
    private static final int SECONDARY_DEVICE = CameraConfig.SECONDARY_USB_CAMERA_DEVICE;
    private static final int SECONDARY_PORT = CameraConfig.SECONDARY_CAMERA_STREAM_PORT;
    
    // Common camera settings
    private static final int CAMERA_WIDTH = CameraConfig.CAMERA_WIDTH;
    private static final int CAMERA_HEIGHT = CameraConfig.CAMERA_HEIGHT;
    private static final int CAMERA_FPS = CameraConfig.CAMERA_FPS;
    
    // Initialization state
    private static boolean isInitialized = false;
    private static boolean primaryConnected = false;
    private static boolean secondaryConnected = false;
    
    // NetworkTables for camera publishing
    private static NetworkTable cameraPublisherTable;
    
    /**
     * Initializes the dual USB camera system.
     * Starts both primary and secondary cameras simultaneously.
     */
    public static void initialize() {
        if (isInitialized) {
            System.out.println("DualUSBCameraServer already initialized");
            return;
        }
        
        try {
            System.out.println("=== INITIALIZING DUAL USB CAMERA SYSTEM ===");
            
            // Set up NetworkTables for camera publishing
            cameraPublisherTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
            
            // Initialize primary camera
            initializePrimaryCamera();
            
            // Initialize secondary camera
            initializeSecondaryCamera();
            
            // Mark as initialized
            isInitialized = true;
            
            // Update SmartDashboard with dual camera status
            updateSmartDashboardStatus();
            
            // Publish camera stream information
            publishCameraStreams();
            
            System.out.println("=== DUAL USB CAMERA SYSTEM INITIALIZED ===");
            System.out.println("Primary Camera: " + (primaryConnected ? "CONNECTED" : "FAILED"));
            System.out.println("Secondary Camera: " + (secondaryConnected ? "CONNECTED" : "FAILED"));
            System.out.println("Team Number: " + CameraConfig.TEAM_NUMBER);
            System.out.println("Primary URL: " + getPrimaryStreamUrl());
            System.out.println("Secondary URL: " + getSecondaryStreamUrl());
            System.out.println("==========================================");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize DualUSBCameraServer: " + e.getMessage());
            isInitialized = false;
        }
    }
    
    /**
     * Initializes the primary USB camera.
     */
    private static void initializePrimaryCamera() {
        try {
            System.out.println("Starting Primary USB Camera (Device " + PRIMARY_DEVICE + ")...");
            
            var primaryCamera = CameraServer.startAutomaticCapture(PRIMARY_CAMERA_NAME, PRIMARY_DEVICE);
            primaryCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            primaryCamera.setFPS(CAMERA_FPS);
            
            primaryConnected = true;
            System.out.println("✅ Primary camera initialized: " + PRIMARY_CAMERA_NAME);
            
        } catch (Exception e) {
            System.err.println("❌ Failed to initialize primary camera: " + e.getMessage());
            primaryConnected = false;
        }
    }
    
    /**
     * Initializes the secondary USB camera.
     */
    private static void initializeSecondaryCamera() {
        try {
            System.out.println("Starting Secondary USB Camera (Device " + SECONDARY_DEVICE + ")...");
            
            // Start automatic capture for secondary camera
            var secondaryCamera = CameraServer.startAutomaticCapture(SECONDARY_CAMERA_NAME, SECONDARY_DEVICE);
            secondaryCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            secondaryCamera.setFPS(CAMERA_FPS);
            
            secondaryConnected = true;
            System.out.println("✅ Secondary camera initialized: " + SECONDARY_CAMERA_NAME);
            
        } catch (Exception e) {
            System.err.println("❌ Failed to initialize secondary camera: " + e.getMessage());
            secondaryConnected = false;
        }
    }
    
    /**
     * Updates SmartDashboard with dual camera status information.
     */
    private static void updateSmartDashboardStatus() {
        SmartDashboard.putBoolean("DualCameras/Initialized", isInitialized);
        SmartDashboard.putBoolean("DualCameras/PrimaryConnected", primaryConnected);
        SmartDashboard.putBoolean("DualCameras/SecondaryConnected", secondaryConnected);
        SmartDashboard.putString("DualCameras/PrimaryName", PRIMARY_CAMERA_NAME);
        SmartDashboard.putString("DualCameras/SecondaryName", SECONDARY_CAMERA_NAME);
        SmartDashboard.putNumber("DualCameras/PrimaryDevice", PRIMARY_DEVICE);
        SmartDashboard.putNumber("DualCameras/SecondaryDevice", SECONDARY_DEVICE);
        SmartDashboard.putString("DualCameras/Resolution", CAMERA_WIDTH + "x" + CAMERA_HEIGHT);
        SmartDashboard.putNumber("DualCameras/FPS", CAMERA_FPS);
        
        // Overall system status
        boolean bothWorking = primaryConnected && secondaryConnected;
        SmartDashboard.putString("DualCameras/Status", 
            bothWorking ? "BOTH CAMERAS WORKING" : 
            primaryConnected ? "PRIMARY ONLY" : 
            secondaryConnected ? "SECONDARY ONLY" : "BOTH FAILED");
    }
    
    /**
     * Publishes camera stream information to NetworkTables.
     */
    private static void publishCameraStreams() {
        if (!isInitialized || cameraPublisherTable == null) {
            return;
        }
        
        try {
            // Publish primary camera stream info
            if (primaryConnected) {
                String primaryStreamInfo = PRIMARY_CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS + ":" + getPrimaryStreamUrl();
                cameraPublisherTable.getEntry(PRIMARY_CAMERA_NAME).setString(primaryStreamInfo);
                SmartDashboard.putString("DualCameras/PrimaryStreamURL", getPrimaryStreamUrl());
                SmartDashboard.putString("DualCameras/PrimaryStreamInfo", primaryStreamInfo);
            }
            
            // Publish secondary camera stream info
            if (secondaryConnected) {
                String secondaryStreamInfo = SECONDARY_CAMERA_NAME + ":" + CAMERA_WIDTH + ":" + CAMERA_HEIGHT + ":" + CAMERA_FPS + ":" + getSecondaryStreamUrl();
                cameraPublisherTable.getEntry(SECONDARY_CAMERA_NAME).setString(secondaryStreamInfo);
                SmartDashboard.putString("DualCameras/SecondaryStreamURL", getSecondaryStreamUrl());
                SmartDashboard.putString("DualCameras/SecondaryStreamInfo", secondaryStreamInfo);
            }
            
        } catch (Exception e) {
            System.err.println("Failed to publish dual camera streams: " + e.getMessage());
        }
    }
    
    /**
     * Gets the stream URL for the primary USB camera.
     * 
     * @return Primary camera stream URL
     */
    public static String getPrimaryStreamUrl() {
        if (!isInitialized || !primaryConnected) {
            return "";
        }
        return "http://roboRIO-" + CameraConfig.TEAM_NUMBER + ".local:" + PRIMARY_PORT + "/stream.mjpg";
    }
    
    /**
     * Gets the stream URL for the secondary USB camera.
     * 
     * @return Secondary camera stream URL
     */
    public static String getSecondaryStreamUrl() {
        if (!isInitialized || !secondaryConnected) {
            return "";
        }
        return "http://roboRIO-" + CameraConfig.TEAM_NUMBER + ".local:" + SECONDARY_PORT + "/stream.mjpg";
    }
    
    /**
     * Checks if the dual camera system is initialized.
     * 
     * @return True if initialized
     */
    public static boolean isInitialized() {
        return isInitialized;
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
     * Checks if both cameras are working.
     * 
     * @return True if both cameras are connected
     */
    public static boolean bothCamerasWorking() {
        return primaryConnected && secondaryConnected;
    }
    
    /**
     * Updates the status information for debugging.
     * This should be called periodically from RobotContainer.periodic().
     */
    public static void updateStatus() {
        if (!isInitialized) {
            return;
        }
        
        try {
            // Update connection status
            SmartDashboard.putBoolean("DualCameras/PrimaryConnected", primaryConnected);
            SmartDashboard.putBoolean("DualCameras/SecondaryConnected", secondaryConnected);
            SmartDashboard.putNumber("DualCameras/LastUpdate", System.currentTimeMillis() / 1000.0);
            
            // Re-publish camera stream info
            publishCameraStreams();
            
            // Provide Shuffleboard setup instructions
            SmartDashboard.putString("DualCameras/ShuffleboardSetup", 
                "DUAL CAMERA SETUP:\\n" +
                "1. Open Shuffleboard\\n" +
                "2. Click '+' to add widget\\n" +
                "3. Select 'Camera Server'\\n" +
                "4. Choose '" + PRIMARY_CAMERA_NAME + "' for primary\\n" +
                "5. Click '+' again\\n" +
                "6. Select 'Camera Server'\\n" +
                "7. Choose '" + SECONDARY_CAMERA_NAME + "' for secondary\\n" +
                "8. Arrange cameras side-by-side");
            
        } catch (Exception e) {
            System.err.println("Error updating DualUSBCameraServer status: " + e.getMessage());
        }
    }
    
    /**
     * Gets comprehensive setup instructions for dual cameras.
     * 
     * @return Setup instructions string
     */
    public static String getSetupInstructions() {
        StringBuilder instructions = new StringBuilder();
        instructions.append("=== DUAL USB CAMERA SETUP INSTRUCTIONS ===\n");
        instructions.append("Status: ").append(bothCamerasWorking() ? "BOTH CAMERAS WORKING" : "PARTIAL SYSTEM").append("\n");
        instructions.append("Primary Camera: ").append(PRIMARY_CAMERA_NAME).append(" (Device ").append(PRIMARY_DEVICE).append(")\n");
        instructions.append("Secondary Camera: ").append(SECONDARY_CAMERA_NAME).append(" (Device ").append(SECONDARY_DEVICE).append(")\n");
        instructions.append("Resolution: ").append(CAMERA_WIDTH).append("x").append(CAMERA_HEIGHT).append("\n");
        instructions.append("FPS: ").append(CAMERA_FPS).append("\n");
        instructions.append("Team Number: ").append(CameraConfig.TEAM_NUMBER).append("\n\n");
        
        instructions.append("STREAM URLs:\n");
        instructions.append("Primary: ").append(getPrimaryStreamUrl()).append("\n");
        instructions.append("Secondary: ").append(getSecondaryStreamUrl()).append("\n\n");
        
        instructions.append("SHUFFLEBOARD SETUP:\n");
        instructions.append("1. Open Shuffleboard and connect to Team ").append(CameraConfig.TEAM_NUMBER).append("\n");
        instructions.append("2. Click '+' to add first widget\n");
        instructions.append("3. Select 'Camera Server'\n");
        instructions.append("4. Choose '").append(PRIMARY_CAMERA_NAME).append("'\n");
        instructions.append("5. Click '+' again to add second widget\n");
        instructions.append("6. Select 'Camera Server'\n");
        instructions.append("7. Choose '").append(SECONDARY_CAMERA_NAME).append("'\n");
        instructions.append("8. Arrange cameras side-by-side for optimal viewing\n");
        instructions.append("9. Add Limelight as third camera with Custom URL: ").append(CameraConfig.LIMELIGHT_STREAM_URL).append("\n\n");
        
        instructions.append("LAYOUT SUGGESTION:\n");
        instructions.append("┌─────────────────┬─────────────────┐\n");
        instructions.append("│ Primary Camera  │ Secondary Camera│\n");
        instructions.append("│ (Driver View)   │ (Intake/Goal)  │\n");
        instructions.append("├─────────────────┼─────────────────┤\n");
        instructions.append("│    Limelight    │   Dashboard     │\n");
        instructions.append("│  (Targeting)    │   (Telemetry)   │\n");
        instructions.append("└─────────────────┴─────────────────┘\n\n");
        
        instructions.append("TROUBLESHOOTING:\n");
        instructions.append("• Check USB camera connections to roboRIO\n");
        instructions.append("• Try different device numbers if cameras not detected\n");
        instructions.append("• Verify cameras work on computer first\n");
        instructions.append("• Check camera permissions on roboRIO\n");
        instructions.append("• Restart robot code if cameras not showing\n");
        instructions.append("• Ensure team number ").append(CameraConfig.TEAM_NUMBER).append(" is correctly configured\n");
        instructions.append("• Check roboRIO web console for camera detection\n");
        instructions.append("============================================");
        
        return instructions.toString();
    }
    
    /**
     * Prints setup instructions to console.
     */
    public static void printSetupInstructions() {
        System.out.println(getSetupInstructions());
    }
    
    /**
     * Gets camera system status summary.
     * 
     * @return Status summary string
     */
    public static String getStatusSummary() {
        if (!isInitialized) {
            return "DualUSBCameraServer: Not initialized";
        }
        
        return String.format("DualUSBCameraServer: Primary=%s, Secondary=%s, Both=%s",
            primaryConnected ? "OK" : "FAIL",
            secondaryConnected ? "OK" : "FAIL", 
            bothCamerasWorking() ? "OK" : "FAIL");
    }
}
