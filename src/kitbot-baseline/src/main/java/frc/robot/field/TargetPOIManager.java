package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Target POI Manager - Specialized manager for target POIs
 * Focuses on shooting locations and strategic targets
 */
public class TargetPOIManager {
    
    private final FieldPOIManager fieldManager;
    
    public TargetPOIManager(double fieldWidth, double fieldHeight, double fieldOfView) {
        this.fieldManager = new FieldPOIManager(fieldWidth, fieldHeight, fieldOfView);
    }
    
    /**
     * Get the best shooting target for current robot position
     */
    public PointOfInterest getBestShootingTarget(Pose2d robotPose, boolean hasFuel) {
        if (!hasFuel) {
            return getNearestFuelSource(robotPose);
        }
        
        List<PointOfInterest> shootingTargets = fieldManager.getPOIsByType(PointOfInterest.POIType.SCORING_LOCATION);
        
        PointOfInterest bestTarget = null;
        double bestScore = -1.0;
        
        for (PointOfInterest target : shootingTargets) {
            if (!target.isActive()) continue;
            
            double score = calculateShootingScore(target, robotPose);
            if (score > bestScore) {
                bestScore = score;
                bestTarget = target;
            }
        }
        
        return bestTarget;
    }
    
    /**
     * Get the nearest fuel source
     */
    public PointOfInterest getNearestFuelSource(Pose2d robotPose) {
        List<PointOfInterest> fuelSources = fieldManager.getPOIsByType(PointOfInterest.POIType.FUEL_SOURCE);
        
        PointOfInterest nearest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (PointOfInterest source : fuelSources) {
            if (!source.isActive()) continue;
            
            double distance = source.distanceTo(robotPose);
            if (distance < minDistance) {
                minDistance = distance;
                nearest = source;
            }
        }
        
        return nearest;
    }
    
    /**
     * Get strategic targets based on game situation
     */
    public List<PointOfInterest> getStrategicTargets(Pose2d robotPose, double timeRemaining, double scoreDifference) {
        List<PointOfInterest> allTargets = fieldManager.getPOIsByType(PointOfInterest.POIType.SCORING_LOCATION);
        
        // Filter and sort based on strategy
        return allTargets.stream()
                .filter(target -> target.isActive())
                .sorted((a, b) -> Double.compare(
                        calculateStrategicScore(b, robotPose, timeRemaining, scoreDifference),
                        calculateStrategicScore(a, robotPose, timeRemaining, scoreDifference)))
                .limit(3) // Top 3 strategic targets
                .collect(java.util.stream.Collectors.toList());
    }
    
    /**
     * Get defensive positions for protecting targets
     */
    public List<PointOfInterest> getDefensivePositions(Pose2d robotPose) {
        List<PointOfInterest> defensivePOIs = fieldManager.getPOIsByType(PointOfInterest.POIType.FIELD_CENTER);
        
        return defensivePOIs.stream()
                .filter(poi -> poi.isActive())
                .filter(poi -> poi.getId().contains("defensive"))
                .sorted((a, b) -> Double.compare(
                        a.distanceTo(robotPose), b.distanceTo(robotPose)))
                .collect(java.util.stream.Collectors.toList());
    }
    
    /**
     * Calculate optimal shooting position for a target
     */
    public Pose2d getOptimalShootingPosition(String targetId, Pose2d currentPose) {
        PointOfInterest target = fieldManager.getPOI(targetId);
        if (target == null || !target.isActive()) {
            return currentPose;
        }
        
        // Get optimal interaction pose from POI
        Pose2d interactionPose = target.getInteractionPose(currentPose);
        
        // Adjust for optimal shooting angle
        Translation2d toTarget = target.getPosition().minus(currentPose.getTranslation());
        double optimalDistance = calculateOptimalShootingDistance(target);
        
        // If current distance is not optimal, adjust position
        double currentDistance = toTarget.getNorm();
        if (Math.abs(currentDistance - optimalDistance) > 0.5) {
            // Move to optimal distance along same angle
            Translation2d adjustment = toTarget.div(currentDistance).times(optimalDistance - currentDistance);
            Translation2d optimalPosition = currentPose.getTranslation().plus(adjustment);
            interactionPose = new Pose2d(optimalPosition, currentPose.getRotation());
        }
        
        return interactionPose;
    }
    
    /**
     * Get all high-value targets for priority scoring
     */
    public List<PointOfInterest> getHighValueTargets(Pose2d robotPose) {
        return fieldManager.getTargetPOIsSorted(robotPose).stream()
                .filter(target -> target.getPriority() == PointOfInterest.POIPriority.CRITICAL)
                .limit(2) // Top 2 critical targets
                .collect(java.util.stream.Collectors.toList());
    }
    
    /**
     * Get backup targets if primary targets are unavailable
     */
    public List<PointOfInterest> getBackupTargets(Pose2d robotPose) {
        return fieldManager.getTargetPOIsSorted(robotPose).stream()
                .filter(target -> target.getPriority() == PointOfInterest.POIPriority.HIGH)
                .filter(target -> target.distanceTo(robotPose) > 2.0) // Not too close
                .limit(3) // Top 3 backup targets
                .collect(java.util.stream.Collectors.toList());
    }
    
    /**
     * Calculate shooting score for a target
     */
    private double calculateShootingScore(PointOfInterest target, Pose2d robotPose) {
        double distance = target.distanceTo(robotPose);
        double value = target.getValue();
        double priority = target.getPriority().getLevel();
        
        // Distance score (closer is better, but not too close)
        double distanceScore = 0.0;
        if (distance >= 1.0 && distance <= 8.0) {
            distanceScore = 10.0 - Math.abs(distance - 3.0); // Optimal at 3 meters
        }
        
        // Combined score
        return distanceScore + value * 2 + priority * 3;
    }
    
    /**
     * Calculate strategic score for target selection
     */
    private double calculateStrategicScore(PointOfInterest target, Pose2d robotPose, 
                                     double timeRemaining, double scoreDifference) {
        double baseScore = calculateShootingScore(target, robotPose);
        
        // Time factor
        double timeFactor = 1.0;
        if (timeRemaining < 10.0) {
            timeFactor = 2.0; // More important when time is short
        } else if (timeRemaining > 15.0) {
            timeFactor = 0.8; // Less important early
        }
        
        // Score difference factor
        double scoreFactor = 1.0;
        if (scoreDifference < -10.0) {
            scoreFactor = 1.5; // Need points badly
        } else if (scoreDifference > 10.0) {
            scoreFactor = 0.7; // Can be more selective
        }
        
        return baseScore * timeFactor * scoreFactor;
    }
    
    /**
     * Calculate optimal shooting distance for target type
     */
    private double calculateOptimalShootingDistance(PointOfInterest target) {
        String targetId = target.getId();
        
        // Different optimal distances for different target types
        if (targetId.contains("scoring")) {
            return 3.0; // Optimal scoring distance
        } else if (targetId.contains("shooting_spot")) {
            return 4.0; // Longer range for shooting spots
        } else if (targetId.contains("strategic")) {
            return 2.5; // Closer for strategic points
        }
        
        return 3.0; // Default optimal distance
    }
    
    /**
     * Update target values based on game state
     */
    public void updateTargetValues(double timeRemaining, double allianceScore, double opponentScore) {
        double scoreDifference = allianceScore - opponentScore;
        
        List<PointOfInterest> allTargets = fieldManager.getPOIsByType(PointOfInterest.POIType.SCORING_LOCATION);
        
        for (PointOfInterest target : allTargets) {
            double newValue = calculateDynamicTargetValue(target, timeRemaining, scoreDifference);
            target.updateValue(newValue);
        }
    }
    
    /**
     * Calculate dynamic target value
     */
    private double calculateDynamicTargetValue(PointOfInterest target, double timeRemaining, double scoreDifference) {
        double baseValue = 5.0; // Base value
        
        // Time urgency
        if (timeRemaining < 5.0) {
            baseValue += 10.0; // Critical when time is almost out
        } else if (timeRemaining < 10.0) {
            baseValue += 5.0; // Important when time is low
        }
        
        // Score pressure
        if (scoreDifference < -15.0) {
            baseValue += 8.0; // Desperate for points
        } else if (scoreDifference < 0) {
            baseValue += 3.0; // Need points
        }
        
        // Position value
        String targetId = target.getId();
        if (targetId.contains("center")) {
            baseValue += 2.0; // Center positions are valuable
        } else if (targetId.contains("shooting_spot")) {
            baseValue += 3.0; // Shooting spots are premium
        }
        
        return baseValue;
    }
    
    /**
     * Get target manager statistics
     */
    public TargetStatistics getTargetStatistics(Pose2d robotPose) {
        List<PointOfInterest> allTargets = fieldManager.getPOIsByType(PointOfInterest.POIType.SCORING_LOCATION);
        List<PointOfInterest> activeTargets = allTargets.stream()
                .filter(PointOfInterest::isActive)
                .collect(java.util.stream.Collectors.toList());
        
        int criticalTargets = (int) activeTargets.stream()
                .filter(target -> target.getPriority() == PointOfInterest.POIPriority.CRITICAL)
                .count();
        
        int highValueTargets = (int) activeTargets.stream()
                .filter(target -> target.getPriority() == PointOfInterest.POIPriority.HIGH)
                .count();
        
        double averageDistance = activeTargets.stream()
                .mapToDouble(target -> target.distanceTo(robotPose))
                .average()
                .orElse(0.0);
        
        PointOfInterest bestTarget = getBestShootingTarget(robotPose, true);
        PointOfInterest nearestFuel = getNearestFuelSource(robotPose);
        
        return new TargetStatistics(
                activeTargets.size(), criticalTargets, highValueTargets,
                averageDistance, bestTarget, nearestFuel);
    }
    
    /**
     * Get the underlying field manager
     */
    public FieldPOIManager getFieldManager() {
        return fieldManager;
    }
    
    /**
     * Target statistics container
     */
    public static class TargetStatistics {
        public final int totalTargets;
        public final int criticalTargets;
        public final int highValueTargets;
        public final double averageDistance;
        public final PointOfInterest bestTarget;
        public final PointOfInterest nearestFuel;
        
        public TargetStatistics(int totalTargets, int criticalTargets, int highValueTargets,
                           double averageDistance, PointOfInterest bestTarget, PointOfInterest nearestFuel) {
            this.totalTargets = totalTargets;
            this.criticalTargets = criticalTargets;
            this.highValueTargets = highValueTargets;
            this.averageDistance = averageDistance;
            this.bestTarget = bestTarget;
            this.nearestFuel = nearestFuel;
        }
        
        @Override
        public String toString() {
            return String.format("TargetStats[Total: %d, Critical: %d, High-Value: %d, AvgDist: %.2f]",
                    totalTargets, criticalTargets, highValueTargets, averageDistance);
        }
    }
}
