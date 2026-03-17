package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Point of Interest (POI) system for FRC field navigation
 * Represents any point on the field that the robot can interact with
 */
public class PointOfInterest {
    
    /**
     * Types of POIs based on their function and interaction methods
     */
    public enum POIType {
        // Reference POIs - for positioning and navigation
        APRIL_TAG("AprilTag", true, false, false),
        FIELD_CORNER("FieldCorner", true, false, false),
        FIELD_CENTER("FieldCenter", true, false, false),
        
        // Target POIs - for robot actions
        SCORING_LOCATION("ScoringLocation", false, true, false),
        FUEL_SOURCE("FuelSource", false, true, false),
        OBSTACLE("Obstacle", false, false, true),
        
        // Special POIs
        ALLIANCE_PARTNER("AlliancePartner", true, true, false),
        OPPONENT_ROBOT("OpponentRobot", true, false, true),
        CLIMBING_ZONE("ClimbingZone", false, true, false);
        
        private final String name;
        private final boolean isReference;
        private final boolean isTarget;
        private final boolean isObstacle;
        
        POIType(String name, boolean isReference, boolean isTarget, boolean isObstacle) {
            this.name = name;
            this.isReference = isReference;
            this.isTarget = isTarget;
            this.isObstacle = isObstacle;
        }
        
        public String getName() { return name; }
        public boolean isReference() { return isReference; }
        public boolean isTarget() { return isTarget; }
        public boolean isObstacle() { return isObstacle; }
    }
    
    /**
     * POI priority levels for decision making
     */
    public enum POIPriority {
        CRITICAL(3),    // Must be processed immediately
        HIGH(2),        // High priority for scoring/avoidance
        MEDIUM(1),      // Normal priority
        LOW(0);         // Can be processed later
        
        private final int level;
        POIPriority(int level) { this.level = level; }
        public int getLevel() { return level; }
    }
    
    // Core POI properties
    private final String id;
    private final POIType type;
    private final Translation2d position;
    private final POIPriority priority;
    
    // Dynamic properties
    private double value;           // Scoring value, importance, etc.
    private boolean isActive;        // Is this POI currently active/available
    private double lastUpdated;     // Timestamp of last update
    private double confidence;      // Confidence in POI position (0-1)
    
    // Interaction properties
    private double interactionRange; // Range at which robot can interact
    private double safetyRange;     // Range to maintain for safety
    private boolean requiresVision;  // Requires vision to locate
    
    // Navigation properties
    private boolean isReachable;    // Can robot physically reach this POI
    private double pathCost;       // Cost to reach this POI
    private Translation2d approachVector; // Preferred approach direction
    
    /**
     * Create a new Point of Interest
     */
    public PointOfInterest(String id, POIType type, Translation2d position, POIPriority priority) {
        this.id = id;
        this.type = type;
        this.position = position;
        this.priority = priority;
        
        // Set default values
        this.value = 0.0;
        this.isActive = true;
        this.lastUpdated = System.currentTimeMillis() / 1000.0;
        this.confidence = 1.0;
        this.interactionRange = 1.0; // 1 meter default
        this.safetyRange = 0.5;     // 0.5 meter default
        this.requiresVision = type == POIType.APRIL_TAG || type == POIType.OPPONENT_ROBOT;
        this.isReachable = true;
        this.pathCost = 0.0;
        this.approachVector = new Translation2d(1.0, 0.0); // Default approach from positive X
    }
    
    /**
     * Create a POI with custom interaction ranges
     */
    public PointOfInterest(String id, POIType type, Translation2d position, 
                        POIPriority priority, double interactionRange, double safetyRange) {
        this(id, type, position, priority);
        this.interactionRange = interactionRange;
        this.safetyRange = safetyRange;
    }
    
    // ========== POSITION AND GEOMETRY ==========
    
    /**
     * Get the position of this POI
     */
    public Translation2d getPosition() {
        return position;
    }
    
    /**
     * Get the position as a Pose2d (with default rotation)
     */
    public Pose2d getPose() {
        return new Pose2d(position, new Rotation2d());
    }
    
    /**
     * Get the position as a Pose2d with specified rotation
     */
    public Pose2d getPose(Rotation2d rotation) {
        return new Pose2d(position, rotation);
    }
    
    /**
     * Calculate distance from this POI to a position
     */
    public double distanceTo(Translation2d otherPosition) {
        return position.getDistance(otherPosition);
    }
    
    /**
     * Calculate distance from this POI to a robot pose
     */
    public double distanceTo(Pose2d robotPose) {
        return distanceTo(robotPose.getTranslation());
    }
    
    /**
     * Calculate angle from this POI to a position
     */
    public Rotation2d angleTo(Translation2d otherPosition) {
        Translation2d delta = otherPosition.minus(position);
        return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    }
    
    /**
     * Calculate angle from this POI to a robot pose
     */
    public Rotation2d angleTo(Pose2d robotPose) {
        return angleTo(robotPose.getTranslation());
    }
    
    // ========== INTERACTION CHECKS ==========
    
    /**
     * Check if robot is within interaction range of this POI
     */
    public boolean isWithinInteractionRange(Pose2d robotPose) {
        return distanceTo(robotPose) <= interactionRange;
    }
    
    /**
     * Check if robot is within safety range of this POI
     */
    public boolean isWithinSafetyRange(Pose2d robotPose) {
        return distanceTo(robotPose) <= safetyRange;
    }
    
    /**
     * Check if this POI is visible from robot position
     */
    public boolean isVisibleFrom(Pose2d robotPose, double fieldOfView) {
        if (!requiresVision) return true;
        
        Rotation2d angleToPOI = angleTo(robotPose);
        Rotation2d robotHeading = robotPose.getRotation();
        
        double angleDifference = Math.abs(angleToPOI.minus(robotHeading).getRadians());
        return angleDifference <= fieldOfView / 2.0;
    }
    
    /**
     * Check if this POI is reachable from robot position
     */
    public boolean isReachableFrom(Pose2d robotPose) {
        if (!isReachable) return false;
        
        // Simple reachability check - can be enhanced with pathfinding
        return distanceTo(robotPose) <= 10.0; // Max field dimension
    }
    
    // ========== REFERENCE POI OPERATIONS ==========
    
    /**
     * Use this POI to update robot odometry
     */
    public OdometryUpdate getOdometryUpdate(Pose2d currentPose, double visionConfidence) {
        if (!type.isReference()) {
            return new OdometryUpdate(currentPose, 0.0, false);
        }
        
        // Calculate pose correction based on POI position
        Translation2d positionError = position.minus(currentPose.getTranslation());
        double distanceError = positionError.getNorm();
        
        // Only update if error is significant and confidence is high
        if (distanceError > 0.1 && confidence * visionConfidence > 0.7) {
            Pose2d correctedPose = new Pose2d(position, currentPose.getRotation());
            return new OdometryUpdate(correctedPose, confidence * visionConfidence, true);
        }
        
        return new OdometryUpdate(currentPose, 0.0, false);
    }
    
    /**
     * Get the preferred approach vector for this POI
     */
    public Translation2d getApproachVector() {
        return approachVector;
    }
    
    /**
     * Set the preferred approach vector for this POI
     */
    public void setApproachVector(Translation2d approachVector) {
        this.approachVector = approachVector;
    }
    
    // ========== TARGET POI OPERATIONS ==========
    
    /**
     * Calculate the optimal pose to interact with this POI
     */
    public Pose2d getInteractionPose(Pose2d currentRobotPose) {
        if (!type.isTarget()) {
            return currentRobotPose; // Cannot interact with non-target POI
        }
        
        // Calculate optimal position (interactionRange away, facing POI)
        Translation2d currentPos = currentRobotPose.getTranslation();
        Translation2d toPOI = position.minus(currentPos);
        
        // Normalize and scale to interaction range
        double distance = toPOI.getNorm();
        if (distance > 0) {
            Translation2d direction = toPOI.div(distance).times(interactionRange);
            Translation2d optimalPosition = position.minus(direction);
            
            // Face towards the POI
            Rotation2d optimalRotation = new Rotation2d(Math.atan2(toPOI.getY(), toPOI.getX()));
            
            return new Pose2d(optimalPosition, optimalRotation);
        }
        
        return currentRobotPose;
    }
    
    /**
     * Check if robot should maintain distance from this POI
     */
    public boolean shouldMaintainDistance() {
        return type.isObstacle() || type == POIType.OPPONENT_ROBOT;
    }
    
    /**
     * Get the recommended distance to maintain from this POI
     */
    public double getRecommendedDistance() {
        if (shouldMaintainDistance()) {
            return safetyRange * 2.0; // Maintain 2x safety range
        }
        return 0.0; // Can approach directly
    }
    
    // ========== DYNAMIC UPDATES ==========
    
    /**
     * Update POI position (for dynamic POIs like robots)
     */
    public void updatePosition(Translation2d newPosition, double newConfidence) {
        // This would update the position in a real implementation
        // For now, just update timestamp and confidence
        this.lastUpdated = System.currentTimeMillis() / 1000.0;
        this.confidence = newConfidence;
    }
    
    /**
     * Update POI value (for scoring optimization)
     */
    public void updateValue(double newValue) {
        this.value = newValue;
        this.lastUpdated = System.currentTimeMillis() / 1000.0;
    }
    
    /**
     * Activate or deactivate this POI
     */
    public void setActive(boolean active) {
        this.isActive = active;
        this.lastUpdated = System.currentTimeMillis() / 1000.0;
    }
    
    /**
     * Update reachability based on current field conditions
     */
    public void updateReachability(boolean reachable, double pathCost) {
        this.isReachable = reachable;
        this.pathCost = pathCost;
        this.lastUpdated = System.currentTimeMillis() / 1000.0;
    }
    
    // ========== UTILITY METHODS ==========
    
    /**
     * Check if this POI is still valid (not too old)
     */
    public boolean isValid(double maxAge) {
        double currentTime = System.currentTimeMillis() / 1000.0;
        return (currentTime - lastUpdated) <= maxAge;
    }
    
    /**
     * Get a score for POI selection (higher is better)
     */
    public double getSelectionScore(Pose2d robotPose) {
        if (!isActive || !isReachable) return -1.0;
        
        double distance = distanceTo(robotPose);
        double score = 0.0;
        
        // Distance score (closer is better)
        score += Math.max(0, 10 - distance);
        
        // Value score
        score += value;
        
        // Priority score
        score += priority.getLevel() * 5;
        
        // Confidence score
        score += confidence * 3;
        
        // Penalty for obstacles
        if (type.isObstacle()) {
            score -= 20; // Avoid obstacles
        }
        
        return score;
    }
    
    // ========== GETTERS AND SETTERS ==========
    
    public String getId() { return id; }
    public POIType getType() { return type; }
    public POIPriority getPriority() { return priority; }
    public double getValue() { return value; }
    public boolean isActive() { return isActive; }
    public double getLastUpdated() { return lastUpdated; }
    public double getConfidence() { return confidence; }
    public double getInteractionRange() { return interactionRange; }
    public double getSafetyRange() { return safetyRange; }
    public boolean requiresVision() { return requiresVision; }
    public boolean isReachable() { return isReachable; }
    public double getPathCost() { return pathCost; }
    
    public void setValue(double value) { this.value = value; }
    public void setConfidence(double confidence) { this.confidence = Math.max(0, Math.min(1, confidence)); }
    public void setInteractionRange(double range) { this.interactionRange = Math.max(0, range); }
    public void setSafetyRange(double range) { this.safetyRange = Math.max(0, range); }
    public void setReachable(boolean reachable) { this.isReachable = reachable; }
    public void setPathCost(double cost) { this.pathCost = Math.max(0, cost); }
    
    @Override
    public String toString() {
        return String.format("POI[%s:%s] at (%.2f, %.2f) - %s, value=%.1f, confidence=%.2f",
                id, type.getName(), position.getX(), position.getY(), 
                isActive ? "ACTIVE" : "INACTIVE", value, confidence);
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
    
    /**
     * Odometry update result from POI reference
     */
    public static class OdometryUpdate {
        private final Pose2d correctedPose;
        private final double confidence;
        private final boolean shouldUpdate;
        
        public OdometryUpdate(Pose2d correctedPose, double confidence, boolean shouldUpdate) {
            this.correctedPose = correctedPose;
            this.confidence = confidence;
            this.shouldUpdate = shouldUpdate;
        }
        
        public Pose2d getCorrectedPose() { return correctedPose; }
        public double getConfidence() { return confidence; }
        public boolean shouldUpdate() { return shouldUpdate; }
    }
}
