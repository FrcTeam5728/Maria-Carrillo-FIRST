package frc.robot.field.strategy;

import frc.robot.field.core.PointOfInterest;
import frc.robot.field.core.POIType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.ArrayList;

/**
 * Strategy system for POI selection and decision making
 * Handles scoring, avoidance, and alliance coordination
 */
public class POIStrategy {
    
    /**
     * Container for scoring strategy
     */
    public static class Scoring {
        private final PointOfInterest target;
        private final double approachDistance;
        private final double estimatedScore;
        private final double confidence;
        
        public Scoring(PointOfInterest target, double approachDistance, double estimatedScore, double confidence) {
            this.target = target;
            this.approachDistance = approachDistance;
            this.estimatedScore = estimatedScore;
            this.confidence = confidence;
        }
        
        public PointOfInterest getTarget() { return target; }
        public double getApproachDistance() { return approachDistance; }
        public double getEstimatedScore() { return estimatedScore; }
        public double getConfidence() { return confidence; }
        public boolean hasTarget() { return target != null; }
        
        @Override
        public String toString() {
            if (target == null) {
                return "Scoring[no target]";
            }
            return String.format("Scoring[target=%s, dist=%.2f, score=%.1f, conf=%.2f]",
                    target.getId(), approachDistance, estimatedScore, confidence);
        }
    }
    
    /**
     * Container for avoidance strategy
     */
    public static class Avoidance {
        private final boolean shouldAvoid;
        private final Translation2d avoidanceVector;
        private final double urgency;
        private final PointOfInterest threat;
        
        public Avoidance(boolean shouldAvoid, Translation2d avoidanceVector, double urgency, PointOfInterest threat) {
            this.shouldAvoid = shouldAvoid;
            this.avoidanceVector = avoidanceVector;
            this.urgency = urgency;
            this.threat = threat;
        }
        
        public boolean shouldAvoid() { return shouldAvoid; }
        public Translation2d getAvoidanceVector() { return avoidanceVector; }
        public double getUrgency() { return urgency; }
        public PointOfInterest getThreat() { return threat; }
        
        @Override
        public String toString() {
            if (!shouldAvoid) {
                return "Avoidance[none]";
            }
            return String.format("Avoidance[threat=%s, vector=(%.2f,%.2f), urgency=%.2f]",
                    threat != null ? threat.getId() : "unknown", 
                    avoidanceVector.getX(), avoidanceVector.getY(), urgency);
        }
    }
    
    /**
     * Container for alliance coordination strategy
     */
    public static class Alliance {
        private final boolean hasAlliance;
        private final Translation2d allianceCenter;
        private final List<PointOfInterest> availableTargets;
        private final String role;
        
        public Alliance(boolean hasAlliance, Translation2d allianceCenter, 
                    List<PointOfInterest> availableTargets, String role) {
            this.hasAlliance = hasAlliance;
            this.allianceCenter = allianceCenter;
            this.availableTargets = new ArrayList<>(availableTargets);
            this.role = role;
        }
        
        public boolean hasAlliance() { return hasAlliance; }
        public Translation2d getAllianceCenter() { return allianceCenter; }
        public List<PointOfInterest> getAvailableTargets() { return new ArrayList<>(availableTargets); }
        public String getRole() { return role; }
        
        @Override
        public String toString() {
            if (!hasAlliance) {
                return "AllianceStrategy[no alliance]";
            }
            return String.format("AllianceStrategy[role=%s, center=(%.2f,%.2f), targets=%d]",
                    role, allianceCenter.getX(), allianceCenter.getY(), availableTargets.size());
        }
    }
    
    /**
     * Calculate best scoring strategy from available targets
     * @param robotPose Current robot pose
     * @param targets List of potential target POIs
     * @param timeRemaining Time remaining in match (seconds)
     * @return Best scoring strategy
     */
    public static Scoring calculateScoringStrategy(Pose2d robotPose, List<PointOfInterest> targets, double timeRemaining) {
        PointOfInterest bestTarget = null;
        double bestScore = -1.0;
        double bestConfidence = 0.0;
        
        for (PointOfInterest target : targets) {
            if (!target.isActive() || !target.getType().isTarget()) {
                continue;
            }
            
            double distance = target.distanceTo(robotPose);
            double value = target.getValue();
            double confidence = target.getConfidence();
            double timeFactor = 1.0 - (timeRemaining / 150.0); // 2.5 minute match
            
            // Score: value + time factor - distance penalty + confidence bonus
            double score = value * 10 + timeFactor * 5 - distance + confidence * 2;
            
            if (score > bestScore) {
                bestScore = score;
                bestTarget = target;
                bestConfidence = confidence;
            }
        }
        
        if (bestTarget == null) {
            return new Scoring(null, 0.0, 0.0, 0.0);
        }
        
        double approachDistance = bestTarget.distanceTo(robotPose);
        double estimatedScore = bestTarget.getValue();
        
        return new Scoring(bestTarget, approachDistance, estimatedScore, bestConfidence);
    }
    
    /**
     * Calculate avoidance strategy for obstacles
     * @param robotPose Current robot pose
     * @param obstacles List of obstacle POIs
     * @return Avoidance strategy
     */
    public static Avoidance calculateAvoidanceStrategy(Pose2d robotPose, List<PointOfInterest> obstacles) {
        if (obstacles.isEmpty()) {
            return new Avoidance(false, new Translation2d(0, 0), 0.0, null);
        }
        
        // Find most immediate threat
        PointOfInterest nearestObstacle = null;
        double minDistance = Double.MAX_VALUE;
        
        for (PointOfInterest obstacle : obstacles) {
            if (!obstacle.isActive()) continue;
            
            double distance = obstacle.distanceTo(robotPose);
            if (distance < minDistance) {
                minDistance = distance;
                nearestObstacle = obstacle;
            }
        }
        
        if (nearestObstacle == null || minDistance > nearestObstacle.getRecommendedDistance()) {
            return new Avoidance(false, new Translation2d(0, 0), 0.0, null);
        }
        
        // Calculate avoidance vector (perpendicular to obstacle direction)
        Translation2d toObstacle = nearestObstacle.getPosition().minus(robotPose.getTranslation());
        Translation2d avoidanceVector = new Translation2d(-toObstacle.getY(), toObstacle.getX());
        avoidanceVector = avoidanceVector.div(avoidanceVector.getNorm());
        
        // Scale avoidance based on how close we are
        double urgency = 1.0 - (minDistance / nearestObstacle.getRecommendedDistance());
        avoidanceVector = avoidanceVector.times(urgency * 2.0); // 2 meter max avoidance
        
        return new Avoidance(true, avoidanceVector, urgency, nearestObstacle);
    }
    
    /**
     * Calculate alliance coordination strategy
     * @param ourPose Current robot pose
     * @param alliancePartners List of alliance partner robots
     * @param targets List of available targets
     * @return Alliance coordination strategy
     */
    public static Alliance calculateAllianceStrategy(Pose2d ourPose, List<PointOfInterest> alliancePartners, List<PointOfInterest> targets) {
        if (alliancePartners.isEmpty()) {
            return new Alliance(false, ourPose.getTranslation(), targets, "solo");
        }
        
        // Find average alliance position
        Translation2d allianceCenter = new Translation2d(0, 0);
        for (PointOfInterest partner : alliancePartners) {
            allianceCenter = allianceCenter.plus(partner.getPosition());
        }
        allianceCenter = allianceCenter.div(alliancePartners.size());
        
        // Determine our role based on position
        String role = determineAllianceRole(ourPose.getTranslation(), allianceCenter);
        
        // Filter targets based on role
        List<PointOfInterest> roleSpecificTargets = filterTargetsByRole(targets, role);
        
        return new Alliance(true, allianceCenter, roleSpecificTargets, role);
    }
    
    /**
     * Determine alliance role based on position
     * @param ourPosition Our robot position
     * @param allianceCenter Alliance center position
     * @return Role string
     */
    private static String determineAllianceRole(Translation2d ourPosition, Translation2d allianceCenter) {
        Translation2d relativePosition = ourPosition.minus(allianceCenter);
        
        if (Math.abs(relativePosition.getX()) > Math.abs(relativePosition.getY())) {
            return relativePosition.getX() > 0 ? "right_wing" : "left_wing";
        } else {
            return relativePosition.getY() > 0 ? "forward" : "defender";
        }
    }
    
    /**
     * Filter targets based on alliance role
     * @param targets All available targets
     * @param role Our alliance role
     * @return Role-specific targets
     */
    private static List<PointOfInterest> filterTargetsByRole(List<PointOfInterest> targets, String role) {
        List<PointOfInterest> filtered = new ArrayList<>();
        
        for (PointOfInterest target : targets) {
            if (!target.isActive()) continue;
            
            // Simple role-based filtering
            switch (role) {
                case "left_wing":
                    // Prefer left-side targets
                    if (target.getPosition().getX() < 0) {
                        filtered.add(target);
                    }
                    break;
                case "right_wing":
                    // Prefer right-side targets
                    if (target.getPosition().getX() > 0) {
                        filtered.add(target);
                    }
                    break;
                case "forward":
                    // Prefer forward targets
                    if (target.getPosition().getY() > 0) {
                        filtered.add(target);
                    }
                    break;
                case "defender":
                    // Prefer defensive positions
                    if (target.getType() == POIType.DEFENSIVE_POSITION) {
                        filtered.add(target);
                    }
                    break;
                default:
                    // No filtering for solo
                    filtered.add(target);
                    break;
            }
        }
        
        return filtered;
    }
}
