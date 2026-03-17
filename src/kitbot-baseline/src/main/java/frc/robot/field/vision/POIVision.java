package frc.robot.field.vision;

import frc.robot.field.core.PointOfInterest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.ArrayList;

/**
 * Vision and odometry correction system for POI-based navigation
 * Handles vision processing, pose correction, and sensor fusion
 */
public class POIVision {
    
    /**
     * Container for vision detection result
     */
    public static class Detection {
        private final PointOfInterest poi;
        private final double confidence;
        private final double timestamp;
        private final Pose2d robotPose;
        
        public Detection(PointOfInterest poi, double confidence, double timestamp, Pose2d robotPose) {
            this.poi = poi;
            this.confidence = confidence;
            this.timestamp = timestamp;
            this.robotPose = robotPose;
        }
        
        public PointOfInterest getPOI() { return poi; }
        public double getConfidence() { return confidence; }
        public double getTimestamp() { return timestamp; }
        public Pose2d getRobotPose() { return robotPose; }
        
        public boolean isRecent(double currentTime, double maxAge) {
            return (currentTime - timestamp) <= maxAge;
        }
        
        @Override
        public String toString() {
            return String.format("Detection[poi=%s, conf=%.2f, age=%.2f]",
                    poi != null ? poi.getId() : "null", confidence, 
                    System.currentTimeMillis() / 1000.0 - timestamp);
        }
    }
    
    /**
     * Container for odometry correction result
     */
    public static class OdometryCorrection {
        private final Pose2d correctedPose;
        private final double confidence;
        private final List<Detection> usedDetections;
        private final double correctionMagnitude;
        
        public OdometryCorrection(Pose2d correctedPose, double confidence, 
                               List<Detection> usedDetections, double correctionMagnitude) {
            this.correctedPose = correctedPose;
            this.confidence = confidence;
            this.usedDetections = new ArrayList<>(usedDetections);
            this.correctionMagnitude = correctionMagnitude;
        }
        
        public Pose2d getCorrectedPose() { return correctedPose; }
        public double getConfidence() { return confidence; }
        public List<Detection> getUsedDetections() { return new ArrayList<>(usedDetections); }
        public double getCorrectionMagnitude() { return correctionMagnitude; }
        
        public boolean isSignificant(double threshold) {
            return correctionMagnitude > threshold;
        }
        
        @Override
        public String toString() {
            return String.format("Correction[conf=%.2f, magnitude=%.3f, detections=%d]",
                    confidence, correctionMagnitude, usedDetections.size());
        }
    }
    
    private final double fieldOfView;
    private final double maxDetectionRange;
    private final List<Detection> recentDetections;
    private final double detectionTimeout;
    
    public POIVision(double fieldOfView, double maxDetectionRange, double detectionTimeout) {
        this.fieldOfView = fieldOfView;
        this.maxDetectionRange = maxDetectionRange;
        this.detectionTimeout = detectionTimeout;
        this.recentDetections = new ArrayList<>();
    }
    
    /**
     * Process vision data and find best POI for odometry correction
     * @param currentPose Current robot pose
     * @param visiblePOIs All POIs that could be visible
     * @param currentTime Current timestamp
     * @return Best POI for odometry correction
     */
    public PointOfInterest findBestOdometryPOI(Pose2d currentPose, List<PointOfInterest> visiblePOIs, double currentTime) {
        PointOfInterest bestPOI = null;
        double bestScore = -1.0;
        
        for (PointOfInterest poi : visiblePOIs) {
            if (!poi.isActive() || !isVisibleFrom(currentPose, poi)) {
                continue;
            }
            
            double distance = poi.distanceTo(currentPose);
            double confidence = calculateVisionConfidence(poi, distance);
            
            // Score: confidence matters more than distance for odometry
            double score = confidence * 10 - distance / maxDetectionRange;
            
            if (score > bestScore) {
                bestScore = score;
                bestPOI = poi;
            }
        }
        
        // Add to recent detections
        if (bestPOI != null) {
            double confidence = calculateVisionConfidence(bestPOI, bestPOI.distanceTo(currentPose));
            recentDetections.add(new Detection(bestPOI, confidence, currentTime, currentPose));
            
            // Clean old detections
            cleanupOldDetections(currentTime);
        }
        
        return bestPOI;
    }
    
    /**
     * Calculate odometry correction using vision detections
     * @param currentOdometry Current odometry pose
     * @param currentTime Current timestamp
     * @return Odometry correction with confidence
     */
    public OdometryCorrection calculateOdometryCorrection(Pose2d currentOdometry, double currentTime) {
        List<Detection> validDetections = getValidDetections(currentTime);
        
        if (validDetections.isEmpty()) {
            return new OdometryCorrection(currentOdometry, 0.0, new ArrayList<>(), 0.0);
        }
        
        // Weighted average of detections for correction
        Translation2d totalTranslation = new Translation2d(0, 0);
        double totalWeight = 0.0;
        
        for (Detection detection : validDetections) {
            double weight = detection.getConfidence();
            Translation2d poiPosition = detection.getPOI().getPosition();
            
            totalTranslation = totalTranslation.plus(poiPosition.times(weight));
            totalWeight += weight;
        }
        
        if (totalWeight == 0) {
            return new OdometryCorrection(currentOdometry, 0.0, new ArrayList<>(), 0.0);
        }
        
        // Calculate corrected pose
        Translation2d averagePosition = totalTranslation.div(totalWeight);
        Pose2d correctedPose = new Pose2d(averagePosition, currentOdometry.getRotation());
        
        // Calculate correction magnitude
        double correctionMagnitude = averagePosition.getDistance(currentOdometry.getTranslation());
        
        // Calculate overall confidence
        double averageConfidence = totalWeight / validDetections.size();
        
        return new OdometryCorrection(correctedPose, averageConfidence, validDetections, correctionMagnitude);
    }
    
    /**
     * Check if POI is visible from current pose
     * @param pose Current robot pose
     * @param poi POI to check
     * @return True if POI is within field of view and range
     */
    private boolean isVisibleFrom(Pose2d pose, PointOfInterest poi) {
        // Check range
        double distance = poi.distanceTo(pose);
        if (distance > maxDetectionRange) {
            return false;
        }
        
        // Check field of view
        return poi.isVisibleFrom(pose, fieldOfView);
    }
    
    /**
     * Calculate vision confidence based on distance and POI properties
     * @param poi POI to evaluate
     * @param distance Distance to POI
     * @return Confidence value [0, 1]
     */
    private double calculateVisionConfidence(PointOfInterest poi, double distance) {
        double distanceConfidence = 1.0 - (distance / maxDetectionRange);
        distanceConfidence = Math.max(0.0, distanceConfidence);
        
        // Combine with POI's inherent confidence
        return (distanceConfidence + poi.getConfidence()) / 2.0;
    }
    
    /**
     * Get valid (recent) detections
     * @param currentTime Current timestamp
     * @return List of valid detections
     */
    private List<Detection> getValidDetections(double currentTime) {
        List<Detection> valid = new ArrayList<>();
        
        for (Detection detection : recentDetections) {
            if (detection.isRecent(currentTime, detectionTimeout)) {
                valid.add(detection);
            }
        }
        
        return valid;
    }
    
    /**
     * Clean up old detections
     * @param currentTime Current timestamp
     */
    private void cleanupOldDetections(double currentTime) {
        recentDetections.removeIf(detection -> !detection.isRecent(currentTime, detectionTimeout));
    }
    
    /**
     * Get recent detections for debugging
     * @return Copy of recent detections
     */
    public List<Detection> getRecentDetections() {
        return new ArrayList<>(recentDetections);
    }
}
