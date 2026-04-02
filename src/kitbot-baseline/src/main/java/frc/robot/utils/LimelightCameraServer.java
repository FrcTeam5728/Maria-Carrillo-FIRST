// Copyright (c) FIRST and other WPilb contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class to integrate Limelight video feed with CameraServer.
 * Provides a simple way to make Limelight stream available in Shuffleboard.
 */
public class LimelightCameraServer {
    
    private static final String LIMELIGHT_IP = "172.22.11.2"; // USB connection to roboRIO
    private static final String STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream";
    private static final String WEB_INTERFACE_URL = "http://" + LIMELIGHT_IP + ":5801";
    
    private static boolean isInitialized = false;
    private static boolean isConnected = false;
    
    /**
     * Initializes the Limelight CameraServer integration.
     * Sets up the camera feed to be available in Shuffleboard.
     */
    public static void initialize() {
        if (isInitialized) {
            return;
        }
        
        try {
            // Test connection to Limelight
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            
            // Check if we can access the Limelight
            double tv = limelightTable.getEntry("tv").getDouble(0.0);
            isConnected = tv >= 0.0; // If we can get a value, connection works
            
            isInitialized = true;
            
            // Update SmartDashboard with status
            SmartDashboard.putBoolean("LimelightCameraServer/Initialized", true);
            SmartDashboard.putBoolean("LimelightCameraServer/Connected", isConnected);
            SmartDashboard.putString("LimelightCameraServer/StreamURL", STREAM_URL);
            SmartDashboard.putString("LimelightCameraServer/WebInterface", WEB_INTERFACE_URL);
            
            System.out.println("=== LIMELIGHT CAMERA SERVER INITIALIZED ===");
            System.out.println("Stream URL: " + STREAM_URL);
            System.out.println("Web Interface: " + WEB_INTERFACE_URL);
            System.out.println("Connected: " + isConnected);
            System.out.println("=======================================");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize Limelight CameraServer: " + e.getMessage());
            SmartDashboard.putBoolean("LimelightCameraServer/Initialized", false);
            SmartDashboard.putString("LimelightCameraServer/Status", "Failed: " + e.getMessage());
        }
    }
    
    /**
     * Gets the stream URL for direct access.
     * This URL can be used to add the camera to Shuffleboard manually.
     * 
     * @return Stream URL
     */
    public static String getStreamUrl() {
        return STREAM_URL;
    }
    
    /**
     * Gets the web interface URL for Limelight configuration.
     * 
     * @return Web interface URL
     */
    public static String getWebInterfaceUrl() {
        return WEB_INTERFACE_URL;
    }
    
    /**
     * Checks if the Limelight CameraServer is initialized.
     * 
     * @return True if initialized
     */
    public static boolean isInitialized() {
        return isInitialized;
    }
    
    /**
     * Checks if the Limelight stream is connected.
     * 
     * @return True if connected
     */
    public static boolean isConnected() {
        return isConnected;
    }
    
    /**
     * Performs a connection test to the Limelight.
     * 
     * @return True if connection successful
     */
    public static boolean testConnection() {
        try {
            // Try to access the Limelight NetworkTable
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            
            // If we can get a value, connection is working
            double tv = limelightTable.getEntry("tv").getDouble(Double.NaN);
            boolean hasTarget = tv > 0.0;
            boolean connectionOk = !Double.isNaN(tv);
            
            isConnected = connectionOk;
            
            SmartDashboard.putBoolean("LimelightCameraServer/ConnectionTest", connectionOk);
            SmartDashboard.putBoolean("LimelightCameraServer/HasTarget", hasTarget);
            SmartDashboard.putBoolean("LimelightCameraServer/Connected", connectionOk);
            
            return connectionOk;
            
        } catch (Exception e) {
            System.err.println("Limelight connection test failed: " + e.getMessage());
            SmartDashboard.putBoolean("LimelightCameraServer/ConnectionTest", false);
            SmartDashboard.putBoolean("LimelightCameraServer/Connected", false);
            isConnected = false;
            return false;
        }
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
            testConnection();
            
            // Update stream info
            SmartDashboard.putString("LimelightCameraServer/StreamURL", STREAM_URL);
            SmartDashboard.putString("LimelightCameraServer/WebInterface", WEB_INTERFACE_URL);
            SmartDashboard.putNumber("LimelightCameraServer/LastUpdate", 
                                   System.currentTimeMillis() / 1000.0);
            
            // Get current Limelight status
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            double tv = limelightTable.getEntry("tv").getDouble(0.0);
            double tx = limelightTable.getEntry("tx").getDouble(0.0);
            double ty = limelightTable.getEntry("ty").getDouble(0.0);
            double ta = limelightTable.getEntry("ta").getDouble(0.0);
            
            SmartDashboard.putBoolean("LimelightCameraServer/HasTarget", tv > 0.0);
            SmartDashboard.putNumber("LimelightCameraServer/HorizontalOffset", tx);
            SmartDashboard.putNumber("LimelightCameraServer/VerticalOffset", ty);
            SmartDashboard.putNumber("LimelightCameraServer/TargetArea", ta);
            
            // Provide troubleshooting info
            SmartDashboard.putString("LimelightCameraServer/Troubleshooting", 
                "1. Access " + STREAM_URL + " in browser" +
                "\\n2. Check Limelight web interface: " + WEB_INTERFACE_URL +
                "\\n3. Enable streaming in Limelight settings" +
                "\\n4. Set pipeline to 1 for video mode");
            
        } catch (Exception e) {
            System.err.println("Error updating Limelight CameraServer status: " + e.getMessage());
        }
    }
    
    /**
     * Gets troubleshooting information.
     * 
     * @return Troubleshooting steps
     */
    public static String getTroubleshootingInfo() {
        StringBuilder info = new StringBuilder();
        info.append("=== LIMELIGHT CAMERA SERVER TROUBLESHOOTING ===\\n");
        info.append("Stream URL: ").append(STREAM_URL).append("\\n");
        info.append("Web Interface: ").append(WEB_INTERFACE_URL).append("\\n\\n");
        info.append("SHUFFLEBOARD SETUP:\\n");
        info.append("1. Open Shuffleboard\\n");
        info.append("2. Click '+' to add widget\\n");
        info.append("3. Select 'Camera Server'\\n");
        info.append("4. Enter stream URL: ").append(STREAM_URL).append("\\n");
        info.append("5. Set camera name to 'Limelight'\\n\\n");
        info.append("COMMON ISSUES:\\n");
        info.append("1. No video in Shuffleboard:\\n");
        info.append("   - Check if stream works in browser\\n");
        info.append("   - Verify Limelight streaming is enabled\\n");
        info.append("   - Set pipeline to 1 (video mode)\\n\\n");
        info.append("2. Connection issues:\\n");
        info.append("   - Ping ").append(LIMELIGHT_IP).append("\\n");
        info.append("   - Check network configuration\\n");
        info.append("   - Verify team number settings\\n\\n");
        info.append("3. Stream quality issues:\\n");
        info.append("   - Reduce resolution to 320x240\\n");
        info.append("   - Lower FPS to 15\\n");
        info.append("   - Check available bandwidth\\n\\n");
        info.append("QUICK FIXES:\\n");
        info.append("• Press X to toggle pipeline modes\\n");
        info.append("• Access Limelight web interface\\n");
        info.append("• Enable 'Stream' checkbox\\n");
        info.append("• Restart robot code\\n");
        info.append("===========================================");
        return info.toString();
    }
    
    /**
     * Prints troubleshooting information to console.
     */
    public static void printTroubleshooting() {
        System.out.println(getTroubleshootingInfo());
    }
}
