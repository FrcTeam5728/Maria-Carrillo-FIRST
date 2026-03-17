package frc.robot.field.manager;

import frc.robot.field.core.PointOfInterest;
import frc.robot.field.interaction.POIInteraction;
import frc.robot.field.navigation.POINavigation;
import frc.robot.field.strategy.POIStrategy;
import frc.robot.field.vision.POIVision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Unified POI Manager - Main interface for all POI operations
 * Coordinates interaction, navigation, strategy, and vision systems
 */
public class UnifiedPOIManager {
    
    // Sub-systems
    private final POIVision vision;
    
    // State
    private PointOfInterest currentTarget;
    private POIInteraction.Strategy currentInteraction;
    private POINavigation.Path currentPath;
    private POIStrategy.Scoring currentScoring;
    private POIStrategy.Avoidance currentAvoidance;
    
    // Configuration
    private final double interactionDistance;
    private final double distanceTolerance;
    private final double headingTolerance;
    
    public UnifiedPOIManager(double interactionDistance, double distanceTolerance, double headingTolerance) {
        this.interactionDistance = interactionDistance;
        this.distanceTolerance = distanceTolerance;
        this.headingTolerance = headingTolerance;
        
        // Initialize sub-systems
        this.vision = new POIVision(Math.toRadians(60), 8.0, 2.0); // 60° FOV, 8m range, 2s timeout
    }
    
    /**
     * Update manager with current robot state
     * @param robotPose Current robot pose
     * @param visiblePOIs List of visible POIs
     * @param obstacles List of obstacle POIs
     * @param targets List of target POIs
     * @param alliancePartners List of alliance partner POIs
     * @param currentTime Current timestamp
     */
    public void update(Pose2d robotPose, List<PointOfInterest> visiblePOIs, 
                   List<PointOfInterest> obstacles, List<PointOfInterest> targets,
                   List<PointOfInterest> alliancePartners, double currentTime) {
        
        // Update vision system
        vision.findBestOdometryPOI(robotPose, visiblePOIs, currentTime);
        
        // Update strategies
        currentScoring = POIStrategy.calculateScoringStrategy(robotPose, targets, getCurrentTimeRemaining(currentTime));
        currentAvoidance = POIStrategy.calculateAvoidanceStrategy(robotPose, obstacles);
        
        // Update navigation
        if (currentScoring.hasTarget()) {
            currentTarget = currentScoring.getTarget();
            currentPath = POINavigation.calculateOptimalPath(robotPose, List.of(currentTarget));
        } else if (currentAvoidance.shouldAvoid()) {
            currentPath = POINavigation.calculateAvoidancePath(robotPose, 
                                                  currentTarget != null ? currentTarget.getInteractionPose(robotPose) : robotPose,
                                                  obstacles);
        }
        
        // Update interaction
        if (currentTarget != null) {
            currentInteraction = POIInteraction.calculateStrategy(robotPose, currentTarget, interactionDistance);
        }
    }
    
    /**
     * Get drive commands for current state
     * @param maxSpeed Maximum drive speed
     * @param maxTurnRate Maximum turn rate
     * @return Drive commands [forwardSpeed, turnSpeed]
     */
    public double[] getDriveCommands(double maxSpeed, double maxTurnRate) {
        // Priority: Avoidance > Interaction > Scoring
        
        if (currentAvoidance.shouldAvoid()) {
            // Avoidance has highest priority
            Translation2d avoidance = currentAvoidance.getAvoidanceVector();
            double avoidanceSpeed = Math.min(maxSpeed, avoidance.getNorm() * currentAvoidance.getUrgency());
            return new double[]{avoidanceSpeed, 0.0}; // Pure avoidance
        }
        
        if (currentInteraction != null && currentInteraction.hasTarget()) {
            // Interaction commands
            return POIInteraction.calculateDriveCommands(currentInteraction, maxSpeed, maxTurnRate);
        }
        
        // Default: no commands
        return new double[]{0.0, 0.0};
    }
    
    /**
     * Check if current interaction is complete
     * @return True if interaction goals are met
     */
    public boolean isInteractionComplete() {
        if (currentInteraction == null) {
            return false;
        }
        
        return POIInteraction.isInteractionComplete(currentInteraction, distanceTolerance, headingTolerance);
    }
    
    /**
     * Get current target POI
     * @return Current target or null if none
     */
    public PointOfInterest getCurrentTarget() {
        return currentTarget;
    }
    
    /**
     * Get current interaction strategy
     * @return Current interaction strategy
     */
    public POIInteraction.Strategy getCurrentInteraction() {
        return currentInteraction;
    }
    
    /**
     * Get current path
     * @return Current navigation path
     */
    public POINavigation.Path getCurrentPath() {
        return currentPath;
    }
    
    /**
     * Get current scoring strategy
     * @return Current scoring strategy
     */
    public POIStrategy.Scoring getCurrentScoring() {
        return currentScoring;
    }
    
    /**
     * Get current avoidance strategy
     * @return Current avoidance strategy
     */
    public POIStrategy.Avoidance getCurrentAvoidance() {
        return currentAvoidance;
    }
    
    /**
     * Get vision corrections
     * @param currentOdometry Current odometry pose
     * @param currentTime Current timestamp
     * @return Odometry correction
     */
    public POIVision.OdometryCorrection getVisionCorrection(Pose2d currentOdometry, double currentTime) {
        return vision.calculateOdometryCorrection(currentOdometry, currentTime);
    }
    
    /**
     * Get recent vision detections
     * @return List of recent detections
     */
    public List<POIVision.Detection> getRecentDetections() {
        return vision.getRecentDetections();
    }
    
    /**
     * Reset manager state
     */
    public void reset() {
        currentTarget = null;
        currentInteraction = null;
        currentPath = null;
        currentScoring = null;
        currentAvoidance = null;
    }
    
    /**
     * Set new target
     * @param target New target POI
     */
    public void setTarget(PointOfInterest target) {
        this.currentTarget = target;
        currentPath = null; // Force recalculation
    }
    
    /**
     * Estimate time remaining in match (simplified)
     * @param currentTime Current timestamp
     * @return Estimated time remaining
     */
    private double getCurrentTimeRemaining(double currentTime) {
        // Simplified: assume 150 second match
        double matchStartTime = System.currentTimeMillis() / 1000.0 - 30.0; // Assume 30 seconds elapsed
        double matchDuration = 150.0; // 2.5 minutes
        return Math.max(0.0, matchDuration - (currentTime - matchStartTime));
    }
    
    /**
     * Get manager status for debugging
     * @return Status string
     */
    public String getStatus() {
        StringBuilder status = new StringBuilder();
        status.append("=== POI Manager Status ===\n");
        
        if (currentTarget != null) {
            status.append(String.format("Target: %s\n", currentTarget.getId()));
        } else {
            status.append("Target: None\n");
        }
        
        if (currentInteraction != null) {
            status.append(String.format("Interaction: %s\n", currentInteraction.toString()));
        }
        
        if (currentAvoidance.shouldAvoid()) {
            status.append(String.format("Avoidance: %s\n", currentAvoidance.toString()));
        }
        
        if (currentScoring.hasTarget()) {
            status.append(String.format("Scoring: %s\n", currentScoring.toString()));
        }
        
        if (currentPath != null) {
            status.append(String.format("Path: %s\n", currentPath.toString()));
        }
        
        return status.toString();
    }
}
