// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/**
 * Configuration constants for camera systems.
 * Contains settings for USB cameras, Limelight, and dual camera setups.
 */
public final class CameraConfig {
    
    /**
     * Camera mode configuration.
     * Set to true to enable dual USB camera support.
     * Set to false for single USB camera mode.
     */
    public static final boolean USE_DUAL_CAMERAS = false;
    
    /**
     * USB Camera device numbers.
     * These correspond to the device numbers assigned by the roboRIO.
     * Typical values: 0 for first camera, 1 for second camera, etc.
     */
    public static final int PRIMARY_USB_CAMERA_DEVICE = 0;
    public static final int SECONDARY_USB_CAMERA_DEVICE = 1;
    
    /**
     * USB Camera resolution and FPS settings.
     */
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final int CAMERA_FPS = 30;
    
    /**
     * USB Camera names for identification in Shuffleboard.
     */
    public static final String PRIMARY_CAMERA_NAME = "DriverCamera";
    public static final String SECONDARY_CAMERA_NAME = "SecondaryCamera";
    
    /**
     * Limelight network configuration.
     */
    public static final String LIMELIGHT_IP = "172.22.11.2"; // USB connection to roboRIO
    public static final String LIMELIGHT_STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    
    /**
     * Team number for camera stream URLs.
     */
    public static final int TEAM_NUMBER = 5728;
    
    /**
     * Camera stream port assignments.
     * Primary camera uses port 1181, secondary uses 1182.
     */
    public static final int PRIMARY_CAMERA_STREAM_PORT = 1181;
    public static final int SECONDARY_CAMERA_STREAM_PORT = 1182;
    
    /**
     * Gets the stream URL for the primary USB camera.
     * 
     * @return Stream URL for primary camera
     */
    public static String getPrimaryCameraStreamUrl() {
        return "http://roboRIO-" + TEAM_NUMBER + ".local:" + PRIMARY_CAMERA_STREAM_PORT + "/stream.mjpg";
    }
    
    /**
     * Gets the stream URL for the secondary USB camera.
     * 
     * @return Stream URL for secondary camera
     */
    public static String getSecondaryCameraStreamUrl() {
        return "http://roboRIO-" + TEAM_NUMBER + ".local:" + SECONDARY_CAMERA_STREAM_PORT + "/stream.mjpg";
    }
    
    /**
     * Gets camera setup instructions based on current configuration.
     * 
     * @return Setup instructions string
     */
    public static String getSetupInstructions() {
        StringBuilder instructions = new StringBuilder();
        
        instructions.append("=== CAMERA CONFIGURATION ===\n");
        instructions.append("Dual Camera Mode: ").append(USE_DUAL_CAMERAS ? "ENABLED" : "DISABLED").append("\n");
        instructions.append("Team Number: ").append(TEAM_NUMBER).append("\n");
        instructions.append("Camera Resolution: ").append(CAMERA_WIDTH).append("x").append(CAMERA_HEIGHT).append("\n");
        instructions.append("Camera FPS: ").append(CAMERA_FPS).append("\n\n");
        
        if (USE_DUAL_CAMERAS) {
            instructions.append("DUAL CAMERA SETUP:\n");
            instructions.append("Primary Camera Device: ").append(PRIMARY_USB_CAMERA_DEVICE).append("\n");
            instructions.append("Secondary Camera Device: ").append(SECONDARY_USB_CAMERA_DEVICE).append("\n");
            instructions.append("Primary Camera Name: ").append(PRIMARY_CAMERA_NAME).append("\n");
            instructions.append("Secondary Camera Name: ").append(SECONDARY_CAMERA_NAME).append("\n");
            instructions.append("Primary Stream URL: ").append(getPrimaryCameraStreamUrl()).append("\n");
            instructions.append("Secondary Stream URL: ").append(getSecondaryCameraStreamUrl()).append("\n\n");
            
            instructions.append("SHUFFLEBOARD SETUP (DUAL CAMERAS):\n");
            instructions.append("1. Open Shuffleboard\n");
            instructions.append("2. Click '+' to add widget\n");
            instructions.append("3. Select 'Camera Server'\n");
            instructions.append("4. Choose '").append(PRIMARY_CAMERA_NAME).append("' for primary camera\n");
            instructions.append("5. Click '+' again to add second widget\n");
            instructions.append("6. Select 'Camera Server'\n");
            instructions.append("7. Choose '").append(SECONDARY_CAMERA_NAME).append("' for secondary camera\n\n");
            
        } else {
            instructions.append("SINGLE CAMERA SETUP:\n");
            instructions.append("USB Camera Device: ").append(PRIMARY_USB_CAMERA_DEVICE).append("\n");
            instructions.append("Camera Name: ").append(PRIMARY_CAMERA_NAME).append("\n");
            instructions.append("Stream URL: ").append(getPrimaryCameraStreamUrl()).append("\n\n");
            
            instructions.append("SHUFFLEBOARD SETUP (SINGLE CAMERA):\n");
            instructions.append("1. Open Shuffleboard\n");
            instructions.append("2. Click '+' to add widget\n");
            instructions.append("3. Select 'Camera Server'\n");
            instructions.append("4. Choose '").append(PRIMARY_CAMERA_NAME).append("'\n\n");
        }
        
        instructions.append("LIMELIGHT SETUP:\n");
        instructions.append("Limelight IP: ").append(LIMELIGHT_IP).append("\n");
        instructions.append("Limelight Stream URL: ").append(LIMELIGHT_STREAM_URL).append("\n");
        instructions.append("Add Camera widget with Custom URL in Shuffleboard\n\n");
        
        instructions.append("TROUBLESHOOTING:\n");
        instructions.append("• Check USB camera connections to roboRIO\n");
        instructions.append("• Try different device numbers (0, 1, 2, 3)\n");
        instructions.append("• Verify cameras work on computer first\n");
        instructions.append("• Check camera permissions on roboRIO\n");
        instructions.append("• Restart robot code if needed\n");
        instructions.append("• Ensure team number ").append(TEAM_NUMBER).append(" is correctly configured\n");
        instructions.append("=============================");
        
        return instructions.toString();
    }
    
    private CameraConfig() {
        // Utility class - prevent instantiation
    }
}
