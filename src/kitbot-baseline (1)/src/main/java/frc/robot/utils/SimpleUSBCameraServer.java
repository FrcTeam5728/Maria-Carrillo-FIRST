// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.CameraConfig;

/**
 * Simple USB camera server for single camera operation.
 * Avoids dual camera complexity and USB bandwidth issues.
 */
public class SimpleUSBCameraServer {
    
    private static UsbCamera camera;
    private static boolean initialized = false;
    private static int currentDevice = 1; // Start with back camera
    
    /**
     * Initialize single USB camera server.
     */
    public static void initialize() {
        initialize(1); // Start with device 1 (back camera)
    }
    
    /**
     * Initialize with specific device number.
     */
    public static void initialize(int deviceNumber) {
        if (initialized) {
            System.out.println("SimpleUSBCameraServer already initialized");
            return;
        }
        
        if (deviceNumber != 0 && deviceNumber != 1) {
            System.err.println("Invalid device: " + deviceNumber + ". Must be 0 or 1.");
            return;
        }
        
        try {
            System.out.println("Initializing simple USB camera server...");
            System.out.println("Starting camera on device " + deviceNumber);
            
            // Remove any existing camera
            CameraServer.removeCamera("SimpleCamera");
            Thread.sleep(500);
            
            // Start camera with specified device
            camera = CameraServer.startAutomaticCapture("SimpleCamera", deviceNumber);
            
            // Configure camera
            Thread.sleep(500); // Wait for camera to initialize
            camera.setResolution(CameraConfig.CAMERA_WIDTH, CameraConfig.CAMERA_HEIGHT);
            camera.setFPS(CameraConfig.CAMERA_FPS);
            
            currentDevice = deviceNumber;
            initialized = true;
            
            // Update SmartDashboard
            SmartDashboard.putBoolean("SimpleUSBCamera/Initialized", true);
            SmartDashboard.putNumber("SimpleUSBCamera/Device", currentDevice);
            SmartDashboard.putString("SimpleUSBCamera/StreamURL", getStreamUrl());
            
            System.out.println("=== SIMPLE USB CAMERA INITIALIZED ===");
            System.out.println("Device: " + currentDevice);
            System.out.println("Resolution: " + CameraConfig.CAMERA_WIDTH + "x" + CameraConfig.CAMERA_HEIGHT);
            System.out.println("FPS: " + CameraConfig.CAMERA_FPS);
            System.out.println("Stream URL: " + getStreamUrl());
            System.out.println("===================================");
            
        } catch (Exception e) {
            System.err.println("Failed to initialize simple USB camera: " + e.getMessage());
            initialized = false;
            SmartDashboard.putBoolean("SimpleUSBCamera/Initialized", false);
            SmartDashboard.putString("SimpleUSBCamera/Error", e.getMessage());
        }
    }
    
    /**
     * Switch to other camera device.
     */
    public static void switchCamera() {
        if (!initialized) {
            System.err.println("Cannot switch - camera not initialized");
            return;
        }
        
        int newDevice = (currentDevice == 0) ? 1 : 0;
        switchToDevice(newDevice);
    }
    
    /**
     * Switch to specific device.
     */
    public static void switchToDevice(int deviceNumber) {
        if (!initialized) {
            System.err.println("Cannot switch - camera not initialized");
            return;
        }
        
        if (deviceNumber != 0 && deviceNumber != 1) {
            System.err.println("Invalid device: " + deviceNumber);
            return;
        }
        
        if (deviceNumber == currentDevice) {
            System.out.println("Already on device " + deviceNumber);
            return;
        }
        
        try {
            System.out.println("Switching to device " + deviceNumber);
            
            // Remove current camera
            CameraServer.removeCamera("SimpleCamera");
            Thread.sleep(500);
            
            // Start new camera
            camera = CameraServer.startAutomaticCapture("SimpleCamera", deviceNumber);
            
            // Configure new camera
            Thread.sleep(500);
            camera.setResolution(CameraConfig.CAMERA_WIDTH, CameraConfig.CAMERA_HEIGHT);
            camera.setFPS(CameraConfig.CAMERA_FPS);
            
            currentDevice = deviceNumber;
            
            // Update SmartDashboard
            SmartDashboard.putNumber("SimpleUSBCamera/Device", currentDevice);
            SmartDashboard.putString("SimpleUSBCamera/StreamURL", getStreamUrl());
            
            System.out.println("Switched to camera " + currentDevice);
            
        } catch (Exception e) {
            System.err.println("Failed to switch camera: " + e.getMessage());
            SmartDashboard.putString("SimpleUSBCamera/Error", e.getMessage());
        }
    }
    
    /**
     * Get current device number.
     */
    public static int getCurrentDevice() {
        return currentDevice;
    }
    
    /**
     * Get stream URL.
     */
    public static String getStreamUrl() {
        if (!initialized) {
            return "";
        }
        return "http://roboRIO-" + CameraConfig.TEAM_NUMBER + ".local:" + 
               CameraConfig.PRIMARY_CAMERA_STREAM_PORT + "/stream.mjpg";
    }
    
    /**
     * Check if initialized.
     */
    public static boolean isInitialized() {
        return initialized;
    }
    
    /**
     * Update status.
     */
    public static void updateStatus() {
        if (!initialized) {
            return;
        }
        
        SmartDashboard.putBoolean("SimpleUSBCamera/Connected", true);
        SmartDashboard.putNumber("SimpleUSBCamera/LastUpdate", System.currentTimeMillis() / 1000.0);
    }
}
