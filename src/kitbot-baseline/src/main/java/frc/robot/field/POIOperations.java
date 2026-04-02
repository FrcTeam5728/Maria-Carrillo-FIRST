package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

/**
 * POI Operations - Utility class for common POI operations
 * Provides high-level operations for working with Points of Interest
 */
public class POIOperations {
    
    /**
     * Container for POI interaction strategy
     */
    public static class POIInteractionStrategy {
        private final PointOfInterest target;
        private final double targetDistance;
        private final Rotation2d targetHeading;
        private final double distanceError;
        private final double headingError;
        
        public POIInteractionStrategy(PointOfInterest target, double targetDistance, Rotation2d targetHeading, 
                                  double distanceError, double headingError) {
            this.target = target;
            this.targetDistance = targetDistance;
            this.targetHeading = targetHeading;
            this.distanceError = distanceError;
            this.headingError = headingError;
        }
        
        public PointOfInterest getTarget() { return target; }
        public double getTargetDistance() { return targetDistance; }
        public Rotation2d getTargetHeading() { return targetHeading; }
        public double getDistanceError() { return distanceError; }
        public double getHeadingError() { return headingError; }
        public boolean hasTarget() { return target != null; }
        
        @Override
        public String toString() {
            if (target == null) {
                return "POIInteraction[no target]";
            }
            return String.format("POIInteraction[target=%s, dist=%.2f, heading=%.1f°, errors=(%.3f,%.1f°)]",
                    target.getId(), targetDistance, Math.toDegrees(targetHeading.getRadians()), 
                    distanceError, Math.toDegrees(headingError));
        }
    }
    
    /**
     * Calculate the optimal path through multiple POIs
     */
    public static POIPath calculateOptimalPath(Pose2d startPose, List<PointOfInterest> pois) {
        if (pois.isEmpty()) {
            return new POIPath(startPose, new Translation2d[0], 0.0);
        }
        
        // Simple nearest-neighbor path planning
        Translation2d[] waypoints = new Translation2d[pois.size()];
        Pose2d currentPose = startPose;
        double totalDistance = 0.0;
        
        for (int i = 0; i < pois.size(); i++) {
            PointOfInterest poi = pois.get(i);
            waypoints[i] = poi.getPosition();
            totalDistance += poi.distanceTo(currentPose);
            currentPose = poi.getInteractionPose(currentPose);
        }
        
        return new POIPath(startPose, waypoints, totalDistance);
    }
    
    /**
     * Find the best POI for odometry correction
     */
    public static PointOfInterest findBestOdometryPOI(Pose2d robotPose, List<PointOfInterest> referencePOIs, double fieldOfView) {
        PointOfInterest bestPOI = null;
        double bestScore = -1.0;
        
        for (PointOfInterest poi : referencePOIs) {
            if (!poi.isActive() || !poi.isVisibleFrom(robotPose, fieldOfView)) {
                continue;
            }
            
            double distance = poi.distanceTo(robotPose);
            double confidence = poi.getConfidence();
            
            // Score: closer is better, but confidence matters more
            double score = confidence * 10 - distance;
            
            if (score > bestScore) {
                bestScore = score;
                bestPOI = poi;
            }
        }
        
        return bestPOI;
    }
    
    /**
     * Calculate avoidance strategy for obstacles
     */
    public static AvoidanceStrategy calculateAvoidanceStrategy(Pose2d robotPose, List<PointOfInterest> obstacles) {
        if (obstacles.isEmpty()) {
            return new AvoidanceStrategy(false, new Translation2d(0, 0), 0.0);
        }
        
        // Find the most immediate threat
        PointOfInterest nearestObstacle = null;
        double minDistance = Double.MAX_VALUE;
        
        for (PointOfInterest obstacle : obstacles) {
            double distance = obstacle.distanceTo(robotPose);
            if (distance < minDistance) {
                minDistance = distance;
                nearestObstacle = obstacle;
            }
        }
        
        if (nearestObstacle == null || minDistance > nearestObstacle.getRecommendedDistance()) {
            return new AvoidanceStrategy(false, new Translation2d(0, 0), 0.0);
        }
        
        // Calculate avoidance vector (perpendicular to obstacle direction)
        Translation2d toObstacle = nearestObstacle.getPosition().minus(robotPose.getTranslation());
        Translation2d avoidanceVector = new Translation2d(-toObstacle.getY(), toObstacle.getX());
        avoidanceVector = avoidanceVector.div(avoidanceVector.getNorm());
        
        // Scale avoidance based on how close we are
        double urgency = 1.0 - (minDistance / nearestObstacle.getRecommendedDistance());
        avoidanceVector = avoidanceVector.times(urgency * 2.0); // 2 meter max avoidance
        
        return new AvoidanceStrategy(true, avoidanceVector, urgency);
    }
    
    /**
     * Calculate scoring strategy for target POIs
     */
    public static ScoringStrategy calculateScoringStrategy(Pose2d robotPose, List<PointOfInterest> targets, double timeRemaining) {
        PointOfInterest bestTarget = null;
        double bestScore = -1.0;
        
        for (PointOfInterest target : targets) {
            if (!target.isActive() || !target.getType().isTarget()) {
                continue;
            }
            
            double distance = target.distanceTo(robotPose);
            double value = target.getValue();
            double timeFactor = 1.0 - (timeRemaining / 20.0); // Higher value when time is short
            
            // Score: value + time factor - distance penalty
            double score = value * 10 + timeFactor * 5 - distance;
            
            if (score > bestScore) {
                bestScore = score;
                bestTarget = target;
            }
        }
        
        if (bestTarget == null) {
            return new ScoringStrategy(null, 0.0, 0.0);
        }
        
        double approachDistance = bestTarget.distanceTo(robotPose);
        double estimatedScore = bestTarget.getValue();
        
        return new ScoringStrategy(bestTarget, approachDistance, estimatedScore);
    }
    
    /**
     * Calculate alliance coordination strategy
     */
    public static AllianceStrategy calculateAllianceStrategy(Pose2d ourPose, List<PointOfInterest> alliancePartners, List<PointOfInterest> targets) {
        if (alliancePartners.isEmpty()) {
            return new AllianceStrategy(false, ourPose.getTranslation(), targets);
        }
        
        // Find average alliance position
        Translation2d allianceCenter = new Translation2d(0, 0);
        for (PointOfInterest partner : alliancePartners) {
            allianceCenter = allianceCenter.plus(partner.getPosition());
        }
        allianceCenter = allianceCenter.div(alliancePartners.size());
        
        // Calculate our role based on position relative to alliance
        double ourDistanceToCenter = ourPose.getTranslation().getDistance(allianceCenter);
        
        // Filter targets based on alliance coordination
        List<PointOfInterest> coordinatedTargets = targets;
        
        // If we're far from alliance center, take distant targets
        if (ourDistanceToCenter > 3.0) {
            coordinatedTargets = targets.stream()
                    .filter(target -> target.distanceTo(ourPose) > 4.0)
                    .collect(java.util.stream.Collectors.toList());
        }
        
        return new AllianceStrategy(true, allianceCenter, coordinatedTargets);
    }
    
    /**
     * Container for POI path information
     */
    public static class POIPath {
        private final Pose2d startPose;
        private final Translation2d[] waypoints;
        private final double totalDistance;
        
        public POIPath(Pose2d startPose, Translation2d[] waypoints, double totalDistance) {
            this.startPose = startPose;
            this.waypoints = waypoints;
            this.totalDistance = totalDistance;
        }
        
        public Pose2d getStartPose() { return startPose; }
        public Translation2d[] getWaypoints() { return waypoints; }
        public double getTotalDistance() { return totalDistance; }
        public int getWaypointCount() { return waypoints.length; }
        
        @Override
        public String toString() {
            return String.format("POIPath[waypoints=%d, distance=%.2f]", waypoints.length, totalDistance);
        }
    }
    
    /**
     * Container for avoidance strategy
     */
    public static class AvoidanceStrategy {
        private final boolean shouldAvoid;
        private final Translation2d avoidanceVector;
        private final double urgency;
        
        public AvoidanceStrategy(boolean shouldAvoid, Translation2d avoidanceVector, double urgency) {
            this.shouldAvoid = shouldAvoid;
            this.avoidanceVector = avoidanceVector;
            this.urgency = urgency;
        }
        
        public boolean shouldAvoid() { return shouldAvoid; }
        public Translation2d getAvoidanceVector() { return avoidanceVector; }
        public double getUrgency() { return urgency; }
        
        @Override
        public String toString() {
            return String.format("Avoidance[avoid=%s, vector=(%.2f,%.2f), urgency=%.2f]",
                    shouldAvoid, avoidanceVector.getX(), avoidanceVector.getY(), urgency);
        }
    }
    
    /**
     * Container for scoring strategy
     */
    public static class ScoringStrategy {
        private final PointOfInterest target;
        private final double approachDistance;
        private final double estimatedScore;
        
        public ScoringStrategy(PointOfInterest target, double approachDistance, double estimatedScore) {
            this.target = target;
            this.approachDistance = approachDistance;
            this.estimatedScore = estimatedScore;
        }
        
        public PointOfInterest getTarget() { return target; }
        public double getApproachDistance() { return approachDistance; }
        public double getEstimatedScore() { return estimatedScore; }
        public boolean hasTarget() { return target != null; }
        
        @Override
        public String toString() {
            if (target == null) {
                return "ScoringStrategy[no target]";
            }
            return String.format("ScoringStrategy[target=%s, distance=%.2f, score=%.1f]",
                    target.getId(), approachDistance, estimatedScore);
        }
    }
    
    /**
     * Container for alliance coordination strategy
     */
    public static class AllianceStrategy {
        private final boolean hasAlliance;
        private final Translation2d allianceCenter;
        private final List<PointOfInterest> recommendedTargets;
        
        public AllianceStrategy(boolean hasAlliance, Translation2d allianceCenter, List<PointOfInterest> recommendedTargets) {
            this.hasAlliance = hasAlliance;
            this.allianceCenter = allianceCenter;
            this.recommendedTargets = recommendedTargets;
        }
        
        public boolean hasAlliance() { return hasAlliance; }
        public Translation2d getAllianceCenter() { return allianceCenter; }
        public List<PointOfInterest> getRecommendedTargets() { return recommendedTargets; }
        
        @Override
        public String toString() {
            return String.format("AllianceStrategy[hasAlliance=%s, center=(%.2f,%.2f), targets=%d]",
                    hasAlliance, allianceCenter.getX(), allianceCenter.getY(), recommendedTargets.size());
        }
    }
}
