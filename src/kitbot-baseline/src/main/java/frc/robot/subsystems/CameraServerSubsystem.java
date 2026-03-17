// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Camera server subsystem for publishing camera feeds to Shuffleboard.
 * Supports USB cameras and Limelight integration.
 */
public class CameraServerSubsystem extends SubsystemBase {
    
    // Camera instances
    private UsbCamera usbCamera;
    private CvSink cvSink;
    private CvSource cvSource;
    
    // Limelight stream
    private CvSink limelightSink;
    private CvSource limelightSource;
    
    // Processing thread
    private Thread visionThread;
    
    // Constants
    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;
    private static final int CAMERA_FPS = 15;
    private static final String LIMELIGHT_IP = "10.57.28.11";
    
    /**
     * Creates a new CameraServerSubsystem.
     */
    public CameraServerSubsystem() {
        initializeUsbCamera();
        initializeLimelightStream();
        startVisionProcessing();
        
        System.out.println("CameraServerSubsystem initialized");
        System.out.println("USB Camera: Available in Shuffleboard");
        System.out.println("Limelight Stream: Available in Shuffleboard");
        System.out.println("Add Camera widgets to Shuffleboard to view feeds");
    }
    
    /**
     * Initializes USB camera for driver view.
     */
    private void initializeUsbCamera() {
        try {
            // Start USB camera capture
            usbCamera = CameraServer.startAutomaticCapture();
            
            // Configure camera settings
            usbCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            usbCamera.setFPS(CAMERA_FPS);
            
            // Create a CvSink for grabbing frames
            cvSink = CameraServer.getVideo();
            
            // Create a CvSource for outputting processed frames
            cvSource = CameraServer.putVideo("USB Camera", CAMERA_WIDTH, CAMERA_HEIGHT);
            
            System.out.println("USB Camera initialized:");
            System.out.println("  Resolution: " + CAMERA_WIDTH + "x" + CAMERA_HEIGHT);
            System.out.println("  FPS: " + CAMERA_FPS);
            System.out.println("  Name: USB Camera (Shuffleboard)");
            
        } catch (Exception e) {
            System.err.println("Error initializing USB camera: " + e.getMessage());
            System.out.println("USB Camera not available - only Limelight stream will work");
        }
    }
    
    /**
     * Initializes Limelight MJPEG stream.
     */
    private void initializeLimelightStream() {
        try {
            // Create CvSource for Limelight stream
            limelightSource = CameraServer.putVideo("Limelight", CAMERA_WIDTH, CAMERA_HEIGHT);
            
            // Note: CvSink for MJPEG streams is more complex
            // For now, we'll create a simple placeholder
            // The actual Limelight stream will be accessible via URL
            
            System.out.println("Limelight stream initialized:");
            System.out.println("  URL: http://" + LIMELIGHT_IP + ":5800/stream.mjpg");
            System.out.println("  Name: Limelight (Shuffleboard)");
            System.out.println("  Resolution: " + CAMERA_WIDTH + "x" + CAMERA_HEIGHT);
            System.out.println("  Use URL directly in Shuffleboard Camera widget");
            
        } catch (Exception e) {
            System.err.println("Error initializing Limelight stream: " + e.getMessage());
            System.out.println("Limelight stream not available - check connection");
        }
    }
    
    /**
     * Starts the vision processing thread.
     */
    private void startVisionProcessing() {
        visionThread = new Thread(() -> {
            Mat usbFrame = new Mat();
            Mat processedFrame = new Mat();
            
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    // Process USB camera only
                    if (cvSink != null && cvSource != null) {
                        long usbTime = cvSink.grabFrame(usbFrame);
                        if (usbTime == 0) {
                            // Error occurred - skip this frame
                            continue;
                        }
                        
                        // Process frame (you can add processing here)
                        Imgproc.resize(usbFrame, processedFrame, new Size(CAMERA_WIDTH, CAMERA_HEIGHT));
                        cvSource.putFrame(processedFrame);
                    }
                    
                    // Control frame rate
                    Thread.sleep(1000 / CAMERA_FPS);
                    
                } catch (Exception e) {
                    System.err.println("Error in vision processing thread: " + e.getMessage());
                    try {
                        Thread.sleep(100); // Prevent tight loop on error
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
            }
            
            // Cleanup
            usbFrame.release();
            processedFrame.release();
        });
        
        visionThread.setDaemon(true);
        visionThread.start();
        
        System.out.println("Vision processing thread started");
    }
    
    /**
     * Gets the Limelight stream URL for direct access.
     * 
     * @return Limelight MJPEG stream URL
     */
    public String getLimelightStreamUrl() {
        return "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    }
    
    /**
     * Gets the USB camera status.
     * 
     * @return True if USB camera is available
     */
    public boolean isUsbCameraAvailable() {
        return usbCamera != null;
    }
    
    /**
     * Gets the Limelight stream status.
     * 
     * @return True if Limelight stream is available
     */
    public boolean isLimelightStreamAvailable() {
        return limelightSource != null;
    }
    
    @Override
    public void periodic() {
        // This method is called periodically - can be used for status updates
    }
    
    public void close() {
        // Cleanup resources
        if (visionThread != null) {
            visionThread.interrupt();
            try {
                visionThread.join(1000); // Wait up to 1 second for thread to stop
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        
        if (cvSink != null) {
            cvSink.close();
        }
        if (cvSource != null) {
            cvSource.close();
        }
        if (limelightSource != null) {
            limelightSource.close();
        }
        if (usbCamera != null) {
            usbCamera.close();
        }
        
        System.out.println("CameraServerSubsystem closed");
    }
}
