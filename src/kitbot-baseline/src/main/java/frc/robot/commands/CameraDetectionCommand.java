// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.CameraFeedStreamer;

/**
 * Command that detects available USB cameras and provides troubleshooting guidance.
 * Useful for finding the correct camera ID when camera connection fails.
 */
public class CameraDetectionCommand extends Command {
    
    private boolean hasRun = false;
    
    /**
     * Creates a new CameraDetectionCommand.
     */
    public CameraDetectionCommand() {
        // No requirements
    }
    
    @Override
    public void initialize() {
        System.out.println("\n=== CAMERA DETECTION AND TROUBLESHOOTING ===");
        
        // Detect available cameras
        String[] cameras = CameraFeedStreamer.detectAvailableCameras();
        
        // Provide troubleshooting guidance
        System.out.println("\n--- CAMERA TROUBLESHOOTING GUIDE ---");
        
        if (cameras.length == 0) {
            System.out.println("❌ NO CAMERAS DETECTED");
            System.out.println("Check the following:");
            System.out.println("1. USB camera is connected to roboRIO");
            System.out.println("2. USB cable is secure and not damaged");
            System.out.println("3. Camera is powered on (if applicable)");
            System.out.println("4. Try different USB port on roboRIO");
            System.out.println("5. Check roboRIO USB device permissions");
            System.out.println("6. Restart roboRIO if camera was recently connected");
        } else {
            System.out.println("✅ DETECTED " + cameras.length + " CAMERA(S):");
            for (String camera : cameras) {
                System.out.println("  • " + camera);
            }
            
            System.out.println("\n--- LIMELIGHT SPECIFIC NOTES ---");
            System.out.println("• Limelight typically appears as /dev/video0 or /dev/video1");
            System.out.println("• If Limelight is detected but streaming fails:");
            System.out.println("  - Check Limelight USB connection");
            System.out.println("  - Verify Limelight is powered (LED should be on)");
            System.out.println("  - Try different USB cable or port");
            System.out.println("  - Check if another application is using the camera");
        }
        
        System.out.println("\n--- CAMERA CONFIGURATION TIPS ---");
        System.out.println("• Update Constants.java to use correct camera ID");
        System.out.println("• Current default: camera ID 0 (/dev/video0)");
        System.out.println("• Change cameraId in CameraFeedStreamer constructor if needed");
        System.out.println("• Limelight should be configured for USB camera mode if using as camera");
        
        System.out.println("\n--- TESTING CAMERA CONNECTION ---");
        System.out.println("• Try pressing camera control buttons:");
        System.out.println("  - Driver POV Up: Enable camera streaming");
        System.out.println("  - Driver POV Down: Disable camera streaming");
        System.out.println("• Watch console for detailed connection messages");
        
        System.out.println("=== CAMERA DETECTION COMPLETE ===\n");
        
        hasRun = true;
    }
    
    @Override
    public boolean isFinished() {
        return hasRun;
    }
}
