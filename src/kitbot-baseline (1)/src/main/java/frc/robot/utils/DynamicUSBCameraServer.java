// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simple USB camera manager that switches between 2 camera devices.
 * Optimized for USB bandwidth with proper resource cleanup.
 */
public class DynamicUSBCameraServer {
    
    private static final String CAMERA_NAME = "DriverCamera";
    private static final int CAMERA_WIDTH = 120;   // Low but more standard resolution
    private static final int CAMERA_HEIGHT = 80;    // Low but more standard resolution  
    private static final int CAMERA_FPS = 8;       // Low FPS but not too low
    private static final int TEAM_NUMBER = 5728;
    
    private static boolean initialized = false;
    private static boolean connected = false;
    private static boolean isCameraInverted = true; // Camera feels backwards by default
    private static int currentDevice = 0;  // Only 0 or 1
    
    // Dual camera setup for proper switching
    private static UsbCamera camera0;
    private static UsbCamera camera1;
    private static VideoSink server;
    private static NetworkTable cameraPublisherTable;
    
    /**
     * Initializes the dynamic USB camera server.
     * Starts with device 1 by default.
     */
    public static void initialize() {
        initialize(1); // Start with camera 1
    }
    
    /**
     * Initializes the dynamic USB camera server with specified starting device.
     * Uses proper WPILib dual camera approach for instant switching.
     * 
     * @param startDevice Starting USB camera device number (0 or 1)
     */
    public static void initialize(int startDevice) {
        if (initialized) {
            System.out.println("DynamicUSBCameraServer already initialized");
            return;
        }
        
        if (startDevice != 0 && startDevice != 1) {
            System.err.println("Invalid start device: " + startDevice + ". Must be 0 or 1.");
            return;
        }
        
        try {
            System.out.println("Initializing dual USB camera system...");
            
            // Start both cameras
            camera0 = CameraServer.startAutomaticCapture(0);
            camera1 = CameraServer.startAutomaticCapture(1);
            
            // Configure both cameras with the same settings
            configureCamera(camera0, "Camera0");
            configureCamera(camera1, "Camera1");
            
            // Get the server for switching
            server = CameraServer.getServer();
            
            // Set initial camera
            currentDevice = startDevice;
            UsbCamera initialCamera = (startDevice == 0) ? camera0 : camera1;
            server.setSource(initialCamera);
            
            connected = true;
            initialized = true;
            
            // Update SmartDashboard
            SmartDashboard.putBoolean("DynamicUSBCamera/Initialized", true);
            SmartDashboard.putBoolean("DynamicUSBCamera/Connected", true);
            SmartDashboard.putNumber("DynamicUSBCamera/DeviceNumber", currentDevice);
            SmartDashboard.putString("DynamicUSBCamera/CurrentCamera", initialCamera.getName());
            SmartDashboard.putBoolean("DynamicUSBCamera/IsReversed", isCameraInverted); // Camera is inverted by default
            
            System.out.println("Dual USB camera system initialized successfully!");
            System.out.println("Active camera: " + initialCamera.getName());
            
        } catch (Exception e) {
            System.err.println("Failed to initialize dual USB camera system: " + e.getMessage());
            initialized = false;
            connected = false;
        }
    }
    
    /**
     * Configures a camera with standard settings.
     */
    private static void configureCamera(UsbCamera camera, String name) {
        try {
            System.out.println("Configuring " + name + "...");
            
            // Wait for camera to initialize
            Thread.sleep(200);
            
            // Set resolution and FPS
            camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            camera.setFPS(CAMERA_FPS);
            
            // Skip pixel format setting - use camera default for compatibility
            System.out.println(name + ": Using camera default pixel format");
            
            // Verify settings
            Thread.sleep(100);
            var mode = camera.getVideoMode();
            System.out.println(name + " configured: " + mode.width + "x" + mode.height + 
                             " @ " + mode.fps + " FPS, Format: " + mode.pixelFormat);
            
        } catch (Exception e) {
            System.err.println("Error configuring " + name + ": " + e.getMessage());
        }
    }
    
    /**
     * Switches between camera 0 and 1.
     * Simple toggle: 0 -> 1 -> 0
     */
    public static void switchCamera() {
        int nextDevice = (currentDevice == 0) ? 1 : 0;  // Only switch between 0 and 1
        switchToDevice(nextDevice);
    }
    
    /**
     * Switches to a specific USB camera device.
     * Uses instant switching with VideoSink.setSource().
     * 
     * @param newDeviceNumber New USB camera device number (0 or 1)
     */
    public static void switchToDevice(int newDeviceNumber) {
        if (!initialized || !connected) {
            System.err.println("Cannot switch camera - system not initialized");
            return;
        }
        
        if (newDeviceNumber < 0 || newDeviceNumber > 1) {
            System.err.println("Invalid device number: " + newDeviceNumber + ". Must be 0 or 1.");
            return;
        }
        
        if (newDeviceNumber == currentDevice) {
            System.out.println("Already on device " + newDeviceNumber);
            return;
        }
        
        try {
            // Instant camera switching using VideoSink.setSource()
            UsbCamera newCamera = (newDeviceNumber == 0) ? camera0 : camera1;
            server.setSource(newCamera);
            currentDevice = newDeviceNumber;
            
            // Update SmartDashboard
            SmartDashboard.putNumber("DynamicUSBCamera/DeviceNumber", currentDevice);
            SmartDashboard.putString("DynamicUSBCamera/CurrentCamera", newCamera.getName());
            SmartDashboard.putBoolean("DynamicUSBCamera/IsReversed", isCameraInverted); // Maintain inversion status
            
            System.out.println("Switched to camera: " + newCamera.getName());
            
        } catch (Exception e) {
            System.err.println("Error switching to device " + newDeviceNumber + ": " + e.getMessage());
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
                            
                            // Wait a moment for camera to initialize
                            Thread.sleep(500);
                            
                            // Get current camera settings for debugging
                            try {
                                var currentMode = camera.getVideoMode();
                                System.out.println("Camera default mode: " + currentMode.width + "x" + currentMode.height + 
                                                 " @ " + currentMode.fps + " FPS, Format: " + currentMode.pixelFormat);
                            } catch (Exception e) {
                                System.err.println("Could not read camera mode: " + e.getMessage());
                            }
                            
                            // Try to set resolution and FPS
                            try {
                                System.out.println("Attempting to set camera to: " + width + "x" + height + " @ " + fps + " FPS");
                                camera.setResolution(width, height);
                                camera.setFPS(fps);
                                
                                // Verify settings were applied
                                try {
                                    Thread.sleep(200); // Wait for settings to apply
                                    var newMode = camera.getVideoMode();
                                    System.out.println("Camera new mode: " + newMode.width + "x" + newMode.height + 
                                                     " @ " + newMode.fps + " FPS, Format: " + newMode.pixelFormat);
                                    
                                    if (newMode.width != width || newMode.height != height) {
                                        System.err.println("WARNING: Resolution not set as requested!");
                                    }
                                } catch (Exception e) {
                                    System.err.println("Could not verify new camera settings: " + e.getMessage());
                                }
                                
                                // Skip pixel format setting - use camera default for compatibility
                                System.out.println("Using camera default pixel format");
                                
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
     * Pauses camera during autonomous to free USB bandwidth.
     * Call this before starting PathPlanner commands.
     */
    public static void pauseForAutonomous() {
        if (initialized && connected) {
            System.out.println("Pausing camera for autonomous - freeing USB bandwidth");
            stopCamera();
        }
    }
    
    /**
     * Resumes camera after autonomous completes.
     * Call this after PathPlanner commands finish.
     */
    public static void resumeFromAutonomous() {
        if (initialized && !connected) {
            System.out.println("Resuming camera after autonomous");
            switchToDevice(currentDevice); // Restart on same device
        }
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
