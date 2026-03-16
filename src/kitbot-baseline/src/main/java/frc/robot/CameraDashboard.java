package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.CameraFeedStreamer;

/**
 * Dashboard utilities for camera streaming.
 * Provides NetworkTables and SmartDashboard integration for camera feeds.
 */
public class CameraDashboard {
    
    private final CameraFeedStreamer streamer;
    
    public CameraDashboard(CameraFeedStreamer streamer) {
        this.streamer = streamer;
    }
    
    /**
     * Updates all camera-related dashboard entries.
     * Call this periodically from Robot.periodic().
     */
    public void updateDashboard() {
        // Update streamer dashboard
        streamer.updateDashboard();
        
        // Add additional camera-specific dashboard entries
        SmartDashboard.putString("Camera/NetworkTables", "/CameraFeed");
        SmartDashboard.putString("Camera/LimelightTable", "/limelight");
        
        // Stream URLs for easy access
        String streamUrl = streamer.getStreamUrl();
        if (!streamUrl.isEmpty()) {
            SmartDashboard.putString("Camera/DirectStream", streamUrl);
            SmartDashboard.putBoolean("Camera/StreamAvailable", true);
        } else {
            SmartDashboard.putString("Camera/DirectStream", "Not available");
            SmartDashboard.putBoolean("Camera/StreamAvailable", false);
        }
        
        // Camera status info
        SmartDashboard.putString("Camera/Status", getCameraStatusText());
        SmartDashboard.putString("Camera/Instructions", getCameraInstructions());
    }
    
    /**
     * Gets human-readable camera status text.
     */
    private String getCameraStatusText() {
        if (streamer.isStreaming()) {
            return "Camera Active";
        } else {
            return "Camera Off";
        }
    }
    
    /**
     * Gets controller instructions for camera control.
     */
    private String getCameraInstructions() {
        return "Driver: D-Pad Up=On, Down=Off | Op: Triggers=Quality";
    }
    
    /**
     * Creates camera shuffleboard layout.
     * Call this once during initialization.
     */
    public void createShuffleboardLayout() {
        // This would create a Shuffleboard tab for camera controls
        // Implementation depends on your Shuffleboard setup
        SmartDashboard.putString("Camera/ShuffleboardTab", "Camera");
    }
}
