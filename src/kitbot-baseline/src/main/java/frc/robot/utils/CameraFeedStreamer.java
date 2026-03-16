package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;

/**
 * Utility class for streaming a single camera feed to NetworkTables.
 * Simplified for one camera setup with easy on/off control.
 */
public class CameraFeedStreamer {
    
    // NetworkTable for camera streaming
    private static final String CAMERA_TABLE_NAME = "CameraFeed";
    private final NetworkTable cameraTable;
    
    // Camera server components
    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource cvSource;
    private Mat imageMat;
    
    // Flag to track if OpenCV is available
    private final boolean openCvAvailable;
    
    // Streaming settings
    private boolean isStreaming = false;
    private int compressionLevel = 50; // JPEG compression (0-100)
    private int targetFps = 15; // Target frames per second
    
    // Camera configuration
    private String cameraName = "MainCamera";
    private int cameraId = 0;
    private int width = 320;
    private int height = 240;
    
    /**
     * Creates a new CameraFeedStreamer.
     */
    public CameraFeedStreamer() {
        this.cameraTable = NetworkTableInstance.getDefault().getTable(CAMERA_TABLE_NAME);
        
        // Test if OpenCV is available
        boolean cvTest = false;
        try {
            Class.forName("org.opencv.core.Mat");
            this.imageMat = new Mat();
            cvTest = true;
            System.out.println("OpenCV native libraries loaded successfully");
        } catch (ClassNotFoundException | NoClassDefFoundError | UnsatisfiedLinkError e) {
            this.imageMat = null;
            cvTest = false;
            System.err.println("OpenCV native libraries not available: " + e.getMessage());
            System.err.println("Camera streaming will work without image processing");
        }
        this.openCvAvailable = cvTest;
        
        // Initialize NetworkTable entries
        cameraTable.getEntry("streaming").setBoolean(false);
        cameraTable.getEntry("camera_name").setString(cameraName);
        cameraTable.getEntry("resolution").setString(width + "x" + height);
        cameraTable.getEntry("fps").setNumber(targetFps);
        cameraTable.getEntry("compression").setNumber(compressionLevel);
    }
    
    /**
     * Starts streaming the USB camera to NetworkTables.
     * Uses the configured camera settings.
     */
    public void startStreaming() {
        try {
            // Stop any existing streaming
            stopStreaming();
            
            // Create USB camera
            camera = CameraServer.startAutomaticCapture(cameraName, cameraId);
            camera.setResolution(width, height);
            camera.setFPS(targetFps);
            
            // Create CvSink and CvSource
            cvSink = CameraServer.getVideo(camera);
            cvSource = CameraServer.putVideo(cameraName + "_Stream", width, height);
            
            // Update NetworkTable entries
            cameraTable.getEntry("streaming").setBoolean(true);
            cameraTable.getEntry("resolution").setString(width + "x" + height);
            cameraTable.getEntry("fps").setNumber(targetFps);
            cameraTable.getEntry("stream_url").setString("http://10.57.28.11:1181/stream.mjpg");
            
            isStreaming = true;
            
            System.out.println("Started camera streaming: " + cameraName + " (" + width + "x" + height + " @ " + targetFps + "fps)");
            System.out.println("Stream URL: http://10.57.28.11:1181/stream.mjpg");
            
            // Start the processing thread
            startProcessingThread();
            
        } catch (Exception e) {
            System.err.println("Failed to start camera streaming: " + e.getMessage());
            stopStreaming();
        }
    }
    
    /**
     * Stops camera streaming.
     */
    public void stopStreaming() {
        if (isStreaming) {
            // Stop camera streaming
            if (cvSink != null) {
                cvSink.close();
                cvSink = null;
            }
            if (cvSource != null) {
                cvSource.close();
                cvSource = null;
            }
            if (camera != null) {
                camera.close();
                camera = null;
            }
            
            isStreaming = false;
            
            // Update NetworkTable entries
            cameraTable.getEntry("streaming").setBoolean(false);
            cameraTable.getEntry("stream_url").setString("");
            
            System.out.println("Stopped camera streaming");
        }
    }
    
    /**
     * Sets the JPEG compression level for streaming.
     * 
     * @param level Compression level (0-100, higher = better quality)
     */
    public void setCompressionLevel(int level) {
        this.compressionLevel = Math.max(0, Math.min(100, level));
        cameraTable.getEntry("compression").setNumber(this.compressionLevel);
        
        System.out.println("Set compression level to: " + this.compressionLevel);
    }
    
    /**
     * Sets the target FPS for streaming.
     * 
     * @param fps Target frames per second
     */
    public void setTargetFps(int fps) {
        this.targetFps = Math.max(1, Math.min(30, fps));
        cameraTable.getEntry("fps").setNumber(this.targetFps);
        
        // Update camera FPS if streaming
        if (camera != null) {
            camera.setFPS(this.targetFps);
        }
        
        System.out.println("Set target FPS to: " + this.targetFps);
    }
    
    /**
     * Gets the current streaming status.
     * 
     * @return True if currently streaming
     */
    public boolean isStreaming() {
        return isStreaming;
    }
    
    /**
     * Checks if OpenCV is available for image processing.
     * 
     * @return True if OpenCV native libraries are loaded
     */
    public boolean isOpenCvAvailable() {
        return openCvAvailable;
    }
    
    /**
     * Gets the stream URL for viewing in a browser.
     * 
     * @return Stream URL string, or empty if not streaming
     */
    public String getStreamUrl() {
        if (isStreaming) {
            return "http://10.57.28.11:1181/stream.mjpg";
        }
        return "";
    }
    
    /**
     * Updates camera status on SmartDashboard.
     */
    public void updateDashboard() {
        SmartDashboard.putBoolean("Camera/Streaming", isStreaming());
        SmartDashboard.putString("Camera/Name", cameraName);
        SmartDashboard.putString("Camera/StreamURL", getStreamUrl());
        SmartDashboard.putNumber("Camera/Compression", compressionLevel);
        SmartDashboard.putNumber("Camera/FPS", targetFps);
        SmartDashboard.putString("Camera/Resolution", width + "x" + height);
    }
    
    /**
     * Starts the image processing thread.
     */
    private void startProcessingThread() {
        if (!openCvAvailable) {
            System.out.println("Camera streaming started without OpenCV processing");
            return; // No processing thread needed if OpenCV not available
        }
        
        Thread processingThread = new Thread(() -> {
            while (!Thread.interrupted() && isStreaming) {
                try {
                    // Grab frame from camera
                    if (cvSink != null && imageMat != null && cvSink.grabFrame(imageMat) == 0) {
                        continue; // Skip if no frame available
                    }
                    
                    // Here you could add image processing
                    // For now, we just pass the frame through
                    
                    // Put processed frame to output
                    if (cvSource != null) {
                        cvSource.putFrame(imageMat);
                    }
                    
                    // Control frame rate
                    Thread.sleep(1000 / targetFps);
                    
                } catch (Exception e) {
                    System.err.println("Error in camera processing thread: " + e.getMessage());
                    break;
                }
            }
        });
        
        processingThread.setDaemon(true);
        processingThread.start();
    }
    
    /**
     * Cleans up resources.
     */
    public void cleanup() {
        stopStreaming();
        if (imageMat != null) {
            imageMat.release();
        }
    }
    
    /**
     * Creates a singleton instance for easy access.
     */
    private static CameraFeedStreamer instance;
    
    public static synchronized CameraFeedStreamer getInstance() {
        if (instance == null) {
            instance = new CameraFeedStreamer();
        }
        return instance;
    }
}
