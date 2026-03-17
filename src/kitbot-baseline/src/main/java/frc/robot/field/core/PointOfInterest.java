package frc.robot.field.core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Core Point of Interest class
 * Represents any significant location on the field
 */
public class PointOfInterest {
    private final String id;
    private final Translation2d position;
    private final POIType type;
    private final double value;
    private final double confidence;
    private final boolean active;
    private final double recommendedDistance;
    
    public PointOfInterest(String id, Translation2d position, POIType type, 
                        double value, double confidence, boolean active, double recommendedDistance) {
        this.id = id;
        this.position = position;
        this.type = type;
        this.value = value;
        this.confidence = confidence;
        this.active = active;
        this.recommendedDistance = recommendedDistance;
    }
    
    // Getters
    public String getId() { return id; }
    public Translation2d getPosition() { return position; }
    public POIType getType() { return type; }
    public double getValue() { return value; }
    public double getConfidence() { return confidence; }
    public boolean isActive() { return active; }
    public double getRecommendedDistance() { return recommendedDistance; }
    
    // Utility methods
    public double distanceTo(Pose2d pose) {
        return position.getDistance(pose.getTranslation());
    }
    
    public boolean isVisibleFrom(Pose2d pose, double fieldOfView) {
        Translation2d toPOI = position.minus(pose.getTranslation());
        double angle = Math.atan2(toPOI.getY(), toPOI.getX());
        double robotHeading = pose.getRotation().getRadians();
        double angleDifference = Math.abs(angle - robotHeading);
        
        // Normalize to [0, 2π]
        while (angleDifference > 2 * Math.PI) angleDifference -= 2 * Math.PI;
        while (angleDifference < 0) angleDifference += 2 * Math.PI;
        
        return angleDifference <= fieldOfView / 2;
    }
    
    public Pose2d getInteractionPose(Pose2d currentPose) {
        Translation2d toPOI = position.minus(currentPose.getTranslation());
        Rotation2d heading = new Rotation2d(Math.atan2(toPOI.getY(), toPOI.getX()));
        
        return new Pose2d(
            position.minus(toPOI.div(toPOI.getNorm()).times(recommendedDistance)),
            heading
        );
    }
    
    @Override
    public String toString() {
        return String.format("POI[id=%s, pos=(%.2f,%.2f), type=%s, value=%.1f, conf=%.2f, active=%s]",
                id, position.getX(), position.getY(), type, value, confidence, active);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        PointOfInterest other = (PointOfInterest) obj;
        return id.equals(other.id);
    }
    
    @Override
    public int hashCode() {
        return id.hashCode();
    }
}
