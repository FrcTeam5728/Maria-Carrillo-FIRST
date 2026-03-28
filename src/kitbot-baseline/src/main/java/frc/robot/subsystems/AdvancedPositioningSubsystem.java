package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Advanced robot positioning system combining AprilTag detection with dead reckoning odometry.
 * Provides absolute positioning when AprilTags are visible, falls back to odometry when not.
 */
public class AdvancedPositioningSubsystem extends SubsystemBase {
    // AprilTag detection
    private AprilTagDetector detector;
    private AprilTagPoseEstimator estimator;
    private AprilTagFieldLayout fieldLayout;
    
    // Camera (using existing camera infrastructure)
    private Object camera; // Can be PhotonCamera or Limelight
    
    // Position tracking
    private Pose2d currentFieldPose = new Pose2d();
    private String positionSource = "Odometry";
    private double confidence = 0.0;
    private long lastAprilTagUpdate = 0;
    
    // Configuration
    private static final double TAG_SIZE_METERS = 0.165; // 6.5 inches
    private static final long TAG_UPDATE_TIMEOUT_MS = 1000; // 1 second timeout
    
    // Position history for filtering
    private final List<Pose2d> positionHistory = new ArrayList<>();
    private static final int HISTORY_SIZE = 10;
    
    // NetworkTables for debugging
    private final Map<String, NetworkTableEntry> debugEntries = new ConcurrentHashMap<>();
    
    /**
     * Creates a new AdvancedPositioningSubsystem.
     */
    public AdvancedPositioningSubsystem() {
        // Initialize AprilTag detector
        initializeAprilTagDetector();
        
        // Load field layout (2024 FRC field)
        loadFieldLayout();
        
        // Initialize debug entries
        initializeDebugEntries();
        
        System.out.println("Advanced Positioning Subsystem initialized with AprilTag + Odometry");
    }
    
    /**
     * Initializes AprilTag detector and pose estimator.
     */
    private void initializeAprilTagDetector() {
        detector = new AprilTagDetector();
        detector.addFamily("36h11"); // FRC 2024+ standard
        
        // Camera matrix (example - should be calibrated for your camera)
        Matrix<N3, N3> cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3())
            .fill(0.0, 0.0, 0.0)
            .fill(0.0, 0.0, 0.0)
            .fill(320.0, 240.0, 1.0) // fx, fy, cx, cy (adjust for your camera)
            .build();
            
        // Distortion coefficients
        double[] distCoeffs = {0, 0, 0, 0, 0};
        
        // Create pose estimator
        estimator = new AprilTagPoseEstimator(
            cameraMatrix,
            distCoeffs,
            new AprilTagPoseEstimator.Config(
                TAG_SIZE_METERS, // tagSize
                16.0,              // xyStdDev
                16.0                 // xyStdDev
            )
        );
    }
    
    /**
     * Loads the AprilTag field layout.
     */
    private void loadFieldLayout() {
        try {
            // Load from deploy directory or create default layout
            fieldLayout = AprilTagFieldLayout.loadResource(
                "/pathplanner/2024-game-tags.json"
            );
        } catch (Exception e) {
            System.out.println("Could not load field layout, using default: " + e.getMessage());
            createDefaultFieldLayout();
        }
    }
    
    /**
     * Creates a default field layout for testing.
     */
    private void createDefaultFieldLayout() {
        fieldLayout = new AprilTagFieldLayout(List.of(
            // Add common field tags (example positions)
            new AprilTag(1, new Pose3d(1.0, 1.0, 0.0, new Rotation3d())),
            new AprilTag(2, new Pose3d(3.0, 1.0, 0.0, new Rotation3d())),
            new AprilTag(3, new Pose3d(5.0, 1.0, 0.0, new Rotation3d())),
            new AprilTag(4, new Pose3d(7.0, 1.0, 0.0, new Rotation3d()))
        ));
    }
    
    /**
     * Initializes NetworkTable entries for debugging.
     */
    private void initializeDebugEntries() {
        // These will appear on Shuffleboard for debugging
        debugEntries.put("FieldX", null);
        debugEntries.put("FieldY", null);
        debugEntries.put("FieldHeading", null);
        debugEntries.put("PositionSource", null);
        debugEntries.put("Confidence", null);
        debugEntries.put("TagsDetected", null);
    }
    
    /**
     * Sets the camera source for AprilTag detection.
     * @param camera PhotonCamera, Limelight, or other camera source
     */
    public void setCamera(Object camera) {
        this.camera = camera;
        System.out.println("Camera set for AprilTag detection: " + camera.getClass().getSimpleName());
    }
    
    /**
     * Updates robot position using AprilTag detection when available.
     * Falls back to odometry when no tags are visible.
     * @param odometryPose Current odometry-based pose
     */
    public void updatePositionWithAprilTags(Pose2d odometryPose) {
        if (camera == null) {
            // No camera available, use odometry only
            updateOdometryOnly(odometryPose);
            return;
        }
        
        try {
            // Get camera frame (implementation depends on camera type)
            Mat frame = getCameraFrame();
            if (frame == null || frame.empty()) {
                updateOdometryOnly(odometryPose);
                return;
            }
            
            // Detect AprilTags
            AprilTagDetection[] detections = detector.detect(frame);
            debugEntries.get("TagsDetected").setNumber(detections.length);
            
            if (detections.length > 0) {
                // Process detections and update position
                processAprilTagDetections(detections, odometryPose);
            } else {
                // No tags detected, use odometry
                updateOdometryOnly(odometryPose);
            }
            
        } catch (Exception e) {
            System.out.println("Error in AprilTag detection: " + e.getMessage());
            updateOdometryOnly(odometryPose);
        }
    }
    
    /**
     * Processes AprilTag detections to determine robot position.
     */
    private void processAprilTagDetections(AprilTagDetection[] detections, Pose2d odometryPose) {
        List<AprilTagPoseEstimate> estimates = new ArrayList<>();
        
        // Convert detections to pose estimates
        for (AprilTagDetection detection : detections) {
            AprilTagPoseEstimate estimate = estimator.estimate(detection);
            if (estimate != null) {
                estimates.add(estimate);
            }
        }
        
        if (!estimates.isEmpty()) {
            // Find best estimate (lowest ambiguity)
            AprilTagPoseEstimate bestEstimate = estimates.get(0);
            for (AprilTagPoseEstimate estimate : estimates) {
                if (estimate.ambiguity < bestEstimate.ambiguity) {
                    bestEstimate = estimate;
                }
            }
            
            // Get field layout position for this tag
            Pose3d tagFieldPose = fieldLayout.getTagPose(bestEstimate.id);
            if (tagFieldPose != null) {
                // Calculate robot pose from tag pose
                Pose3d robotPose3d = calculateRobotPoseFromTag(bestEstimate, tagFieldPose);
                Pose2d newFieldPose = robotPose3d.toPose2d();
                
                // Validate the new position
                if (isValidPosition(newFieldPose, odometryPose)) {
                    updateFieldPosition(newFieldPose, "AprilTag " + bestEstimate.id, 
                        1.0 - bestEstimate.ambiguity);
                    lastAprilTagUpdate = System.currentTimeMillis();
                    
                    System.out.printf("AprilTag %d detected - Field: (%.2f, %.2f, %.1f°)%n", 
                        bestEstimate.id, newFieldPose.getX(), newFieldPose.getY(), 
                        newFieldPose.getRotation().getDegrees());
                }
            }
        } else {
            updateOdometryOnly(odometryPose);
        }
    }
    
    /**
     * Calculates robot pose from detected tag pose.
     */
    private Pose3d calculateRobotPoseFromTag(AprilTagPoseEstimate tagEstimate, Pose3d tagFieldPose) {
        // Transform from tag pose to robot pose
        // tagPose is robot-to-tag transformation
        // tagFieldPose is field-to-tag transformation
        // robotPose = tagFieldPose * tagPose.inverse()
        
        Transform3d tagToRobot = tagEstimate.pose.inverse();
        return tagFieldPose.transformBy(tagToRobot);
    }
    
    /**
     * Validates that a new position is reasonable compared to odometry.
     */
    private boolean isValidPosition(Pose2d newPose, Pose2d odometryPose) {
        // Check if position is too far from odometry (indicates error)
        double distance = odometryPose.getTranslation().getDistance(newPose.getTranslation());
        double angleDiff = Math.abs(odometryPose.getRotation().minus(newPose.getRotation()).getDegrees());
        
        // Allow reasonable deviation (adjust based on your needs)
        double maxDistance = 2.0; // 2 meters max deviation
        double maxAngleDiff = 45.0; // 45 degrees max deviation
        
        return distance < maxDistance && angleDiff < maxAngleDiff;
    }
    
    /**
     * Updates position using only odometry (when no AprilTags visible).
     */
    private void updateOdometryOnly(Pose2d odometryPose) {
        updateFieldPosition(odometryPose, "Odometry", 
            Math.max(0.0, confidence - 0.01)); // Gradually reduce confidence
    }
    
    /**
     * Updates the field position and related data.
     */
    private void updateFieldPosition(Pose2d pose, String source, double conf) {
        currentFieldPose = pose;
        positionSource = source;
        confidence = conf;
        
        // Add to history for filtering
        positionHistory.add(pose);
        if (positionHistory.size() > HISTORY_SIZE) {
            positionHistory.remove(0);
        }
        
        // Update debug entries
        updateDebugEntries();
    }
    
    /**
     * Updates NetworkTable entries for debugging.
     */
    private void updateDebugEntries() {
        debugEntries.get("FieldX").setDouble(currentFieldPose.getX());
        debugEntries.get("FieldY").setDouble(currentFieldPose.getY());
        debugEntries.get("FieldHeading").setDouble(currentFieldPose.getRotation().getDegrees());
        debugEntries.get("PositionSource").setString(positionSource);
        debugEntries.get("Confidence").setDouble(confidence);
    }
    
    /**
     * Gets the current field-relative pose.
     */
    public Pose2d getFieldPose() {
        return currentFieldPose;
    }
    
    /**
     * Gets the current position source.
     */
    public String getPositionSource() {
        return positionSource;
    }
    
    /**
     * Gets the current positioning confidence.
     */
    public double getConfidence() {
        return confidence;
    }
    
    /**
     * Gets whether AprilTags have been recently detected.
     */
    public boolean hasRecentAprilTagDetection() {
        return (System.currentTimeMillis() - lastAprilTagUpdate) < TAG_UPDATE_TIMEOUT_MS;
    }
    
    /**
     * Gets filtered position (average of recent positions).
     */
    public Pose2d getFilteredPosition() {
        if (positionHistory.isEmpty()) {
            return currentFieldPose;
        }
        
        // Simple moving average
        double sumX = 0, sumY = 0, sumCos = 0, sumSin = 0;
        int count = Math.min(positionHistory.size(), 5); // Use last 5 positions
        
        for (int i = positionHistory.size() - count; i < positionHistory.size(); i++) {
            Pose2d pose = positionHistory.get(i);
            sumX += pose.getX();
            sumY += pose.getY();
            sumCos += Math.cos(pose.getRotation().getRadians());
            sumSin += Math.sin(pose.getRotation().getRadians());
        }
        
        double avgX = sumX / count;
        double avgY = sumY / count;
        double avgAngle = Math.atan2(sumSin, sumCos);
        
        return new Pose2d(avgX, avgY, Rotation2d.fromRadians(avgAngle));
    }
    
    /**
     * Resets position to known field location.
     */
    public void resetPosition(Pose2d pose) {
        currentFieldPose = pose;
        positionSource = "Manual Reset";
        confidence = 1.0;
        positionHistory.clear();
        positionHistory.add(pose);
        updateDebugEntries();
        
        System.out.printf("Position reset to: (%.2f, %.2f, %.1f°)%n", 
            pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
    
    /**
     * Gets camera frame (implementation depends on camera type).
     */
    private Mat getCameraFrame() {
        try {
            if (camera != null) {
                // Handle different camera types
                String className = camera.getClass().getSimpleName();
                
                if (className.contains("Photon")) {
                    // PhotonVision camera
                    var result = (Object) camera.getClass().getMethod("getLatestResult").invoke(camera);
                    var image = (Mat) result.getClass().getMethod("getCameraImage").invoke(result);
                    return image;
                    
                } else if (className.contains("Limelight")) {
                    // Limelight camera
                    var pipeline = (Object) camera.getClass().getMethod("getPipeline").invoke(camera);
                    var snapshot = (Object) pipeline.getClass().getMethod("getSnapshot").invoke(pipeline);
                    return (Mat) snapshot.getClass().getMethod("getMat").invoke(snapshot);
                    
                } else {
                    // Generic camera (implement as needed)
                    System.out.println("Unsupported camera type: " + className);
                    return new Mat(); // Empty frame
                }
            }
        } catch (Exception e) {
            System.out.println("Error getting camera frame: " + e.getMessage());
            return new Mat(); // Empty frame
        }
    }
    
    @Override
    public void periodic() {
        // This method is called periodically to maintain the subsystem
        long currentTime = System.currentTimeMillis();
        
        // Check for AprilTag timeout
        if (positionSource.startsWith("AprilTag") && 
            (currentTime - lastAprilTagUpdate) > TAG_UPDATE_TIMEOUT_MS) {
            System.out.println("AprilTag timeout, switching to odometry");
            confidence = Math.max(0.0, confidence - 0.05);
        }
    }
}
