package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Field POI Manager - Manages all Points of Interest on the field
 * Provides operations for POI discovery, selection, and interaction
 */
public class FieldPOIManager {
    
    // Storage for all POIs
    private final Map<String, PointOfInterest> allPOIs;
    private final List<PointOfInterest> activePOIs;
    private final Map<PointOfInterest.POIType, List<PointOfInterest>> poisByType;
    
    // Field properties
    private final double fieldWidth;
    private final double fieldHeight;
    private final double fieldOfView;
    
    // Cache for frequently accessed POIs
    private double lastCacheUpdate;
    
    /**
     * Create a new Field POI Manager
     */
    public FieldPOIManager(double fieldWidth, double fieldHeight, double fieldOfView) {
        this.fieldWidth = fieldWidth;
        this.fieldHeight = fieldHeight;
        this.fieldOfView = fieldOfView;
        
        this.allPOIs = new HashMap<>();
        this.activePOIs = new ArrayList<>();
        this.poisByType = new HashMap<>();
        
        this.lastCacheUpdate = 0.0;
        
        initializeFieldPOIs();
    }
    
    /**
     * Initialize standard field POIs (AprilTags, scoring locations, etc.)
     */
    private void initializeFieldPOIs() {
        // Add AprilTag reference POIs (2026 REBUILT field)
        addPOI(new PointOfInterest("april_tag_1", PointOfInterest.POIType.APRIL_TAG, 
                new Translation2d(0.0, 0.0), PointOfInterest.POIPriority.HIGH));
        addPOI(new PointOfInterest("april_tag_2", PointOfInterest.POIType.APRIL_TAG, 
                new Translation2d(fieldWidth, 0.0), PointOfInterest.POIPriority.HIGH));
        addPOI(new PointOfInterest("april_tag_3", PointOfInterest.POIType.APRIL_TAG, 
                new Translation2d(fieldWidth, fieldHeight), PointOfInterest.POIPriority.HIGH));
        addPOI(new PointOfInterest("april_tag_4", PointOfInterest.POIType.APRIL_TAG, 
                new Translation2d(0.0, fieldHeight), PointOfInterest.POIPriority.HIGH));
        addPOI(new PointOfInterest("april_tag_5", PointOfInterest.POIType.APRIL_TAG, 
                new Translation2d(fieldWidth/2, fieldHeight/2), PointOfInterest.POIPriority.HIGH));
        
        // Add field reference POIs
        addPOI(new PointOfInterest("field_center", PointOfInterest.POIType.FIELD_CENTER, 
                new Translation2d(fieldWidth/2, fieldHeight/2), PointOfInterest.POIPriority.MEDIUM));
        addPOI(new PointOfInterest("corner_1", PointOfInterest.POIType.FIELD_CORNER, 
                new Translation2d(0.0, 0.0), PointOfInterest.POIPriority.LOW));
        addPOI(new PointOfInterest("corner_2", PointOfInterest.POIType.FIELD_CORNER, 
                new Translation2d(fieldWidth, 0.0), PointOfInterest.POIPriority.LOW));
        addPOI(new PointOfInterest("corner_3", PointOfInterest.POIType.FIELD_CORNER, 
                new Translation2d(fieldWidth, fieldHeight), PointOfInterest.POIPriority.LOW));
        addPOI(new PointOfInterest("corner_4", PointOfInterest.POIType.FIELD_CORNER, 
                new Translation2d(0.0, fieldHeight), PointOfInterest.POIPriority.LOW));
        
        // Add scoring locations (2026 REBUILT game elements)
        addPOI(new PointOfInterest("scoring_left", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(1.0, fieldHeight/2), PointOfInterest.POIPriority.CRITICAL, 1.5, 0.5));
        addPOI(new PointOfInterest("scoring_center", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(fieldWidth/2, fieldHeight/2), PointOfInterest.POIPriority.CRITICAL, 1.5, 0.5));
        addPOI(new PointOfInterest("scoring_right", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(fieldWidth - 1.0, fieldHeight/2), PointOfInterest.POIPriority.CRITICAL, 1.5, 0.5));
        
        // Add high-value shooting targets (advantageous positions)
        addPOI(new PointOfInterest("shooting_spot_left", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(2.5, 2.0), PointOfInterest.POIPriority.HIGH, 2.0, 0.8));
        addPOI(new PointOfInterest("shooting_spot_center", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(fieldWidth/2, 1.5), PointOfInterest.POIPriority.HIGH, 2.0, 0.8));
        addPOI(new PointOfInterest("shooting_spot_right", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(fieldWidth - 2.5, 2.0), PointOfInterest.POIPriority.HIGH, 2.0, 0.8));
        
        // Add strategic positions (defensive and offensive)
        addPOI(new PointOfInterest("strategic_point_1", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(3.0, fieldHeight - 2.0), PointOfInterest.POIPriority.MEDIUM, 1.8, 0.6));
        addPOI(new PointOfInterest("strategic_point_2", PointOfInterest.POIType.SCORING_LOCATION, 
                new Translation2d(fieldWidth - 3.0, fieldHeight - 2.0), PointOfInterest.POIPriority.MEDIUM, 1.8, 0.6));
        
        // Add fuel sources
        addPOI(new PointOfInterest("fuel_source_1", PointOfInterest.POIType.FUEL_SOURCE, 
                new Translation2d(2.0, 1.0), PointOfInterest.POIPriority.HIGH, 2.0, 1.0));
        addPOI(new PointOfInterest("fuel_source_2", PointOfInterest.POIType.FUEL_SOURCE, 
                new Translation2d(fieldWidth - 2.0, 1.0), PointOfInterest.POIPriority.HIGH, 2.0, 1.0));
        addPOI(new PointOfInterest("fuel_source_3", PointOfInterest.POIType.FUEL_SOURCE, 
                new Translation2d(fieldWidth/2, fieldHeight - 1.0), PointOfInterest.POIPriority.HIGH, 2.0, 1.0));
        
        // Add climbing zones
        addPOI(new PointOfInterest("climb_zone_left", PointOfInterest.POIType.CLIMBING_ZONE, 
                new Translation2d(0.5, fieldHeight - 0.5), PointOfInterest.POIPriority.MEDIUM, 1.0, 0.5));
        addPOI(new PointOfInterest("climb_zone_right", PointOfInterest.POIType.CLIMBING_ZONE, 
                new Translation2d(fieldWidth - 0.5, fieldHeight - 0.5), PointOfInterest.POIPriority.MEDIUM, 1.0, 0.5));
        
        // Add important field locations for positioning
        addPOI(new PointOfInterest("center_line", PointOfInterest.POIType.FIELD_CENTER, 
                new Translation2d(fieldWidth/2, 0.0), PointOfInterest.POIPriority.MEDIUM, 1.0, 0.5));
        addPOI(new PointOfInterest("midfield", PointOfInterest.POIType.FIELD_CENTER, 
                new Translation2d(fieldWidth/2, fieldHeight/2), PointOfInterest.POIPriority.MEDIUM, 1.0, 0.5));
        
        // Add defensive positions
        addPOI(new PointOfInterest("defensive_left", PointOfInterest.POIType.FIELD_CENTER, 
                new Translation2d(1.0, 1.0), PointOfInterest.POIPriority.MEDIUM, 1.2, 0.7));
        addPOI(new PointOfInterest("defensive_right", PointOfInterest.POIType.FIELD_CENTER, 
                new Translation2d(fieldWidth - 1.0, 1.0), PointOfInterest.POIPriority.MEDIUM, 1.2, 0.7));
    }
    
    // ========== POI MANAGEMENT ==========
    
    /**
     * Add a new POI to the manager
     */
    public void addPOI(PointOfInterest poi) {
        allPOIs.put(poi.getId(), poi);
        
        // Add to type-specific list
        poisByType.computeIfAbsent(poi.getType(), k -> new ArrayList<>()).add(poi);
        
        // Add to active list if active
        if (poi.isActive()) {
            activePOIs.add(poi);
        }
        
        // Clear cache
        clearCache();
    }
    
    /**
     * Remove a POI from the manager
     */
    public void removePOI(String poiId) {
        PointOfInterest poi = allPOIs.remove(poiId);
        if (poi != null) {
            activePOIs.remove(poi);
            poisByType.getOrDefault(poi.getType(), new ArrayList<>()).remove(poi);
            clearCache();
        }
    }
    
    /**
     * Update an existing POI
     */
    public void updatePOI(String poiId, Translation2d newPosition, double confidence) {
        PointOfInterest poi = allPOIs.get(poiId);
        if (poi != null) {
            poi.updatePosition(newPosition, confidence);
            clearCache();
        }
    }
    
    /**
     * Add dynamic POI (e.g., detected robot)
     */
    public void addDynamicPOI(String id, PointOfInterest.POIType type, 
                           Translation2d position, double confidence, double lifetime) {
        PointOfInterest poi = new PointOfInterest(id, type, position, PointOfInterest.POIPriority.MEDIUM);
        poi.setConfidence(confidence);
        poi.setActive(true);
        
        addPOI(poi);
        
        // Schedule removal after lifetime (in a real implementation, use timer)
        // schedulePOIRemoval(id, lifetime);
    }
    
    // ========== REFERENCE POI OPERATIONS ==========
    
    /**
     * Get the best reference POI for odometry update
     */
    public PointOfInterest getBestReferencePOI(Pose2d robotPose) {
        // Update cache if needed
        updateCacheIfNeeded();
        
        List<PointOfInterest> referencePOIs = getPOIsByType(PointOfInterest.POIType.APRIL_TAG);
        
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
     * Get odometry update from all visible reference POIs
     */
    public PointOfInterest.OdometryUpdate getOdometryUpdate(Pose2d currentPose) {
        PointOfInterest bestPOI = getBestReferencePOI(currentPose);
        if (bestPOI != null) {
            return bestPOI.getOdometryUpdate(currentPose, 0.8); // Vision confidence
        }
        
        return new PointOfInterest.OdometryUpdate(currentPose, 0.0, false);
    }
    
    /**
     * Get all reference POIs visible from robot position
     */
    public List<PointOfInterest> getVisibleReferencePOIs(Pose2d robotPose) {
        return getPOIsByType(PointOfInterest.POIType.APRIL_TAG).stream()
                .filter(poi -> poi.isActive() && poi.isVisibleFrom(robotPose, fieldOfView))
                .collect(Collectors.toList());
    }
    
    // ========== TARGET POI OPERATIONS ==========
    
    /**
     * Get the best target POI for the robot to interact with
     */
    public PointOfInterest getBestTargetPOI(Pose2d robotPose) {
        updateCacheIfNeeded();
        
        PointOfInterest bestPOI = null;
        double bestScore = -1.0;
        
        for (PointOfInterest poi : activePOIs) {
            if (!poi.getType().isTarget()) {
                continue;
            }
            
            double score = poi.getSelectionScore(robotPose);
            if (score > bestScore) {
                bestScore = score;
                bestPOI = poi;
            }
        }
        
        return bestPOI;
    }
    
    /**
     * Get all target POIs sorted by priority
     */
    public List<PointOfInterest> getTargetPOIsSorted(Pose2d robotPose) {
        return activePOIs.stream()
                .filter(poi -> poi.getType().isTarget())
                .sorted((a, b) -> Double.compare(
                        b.getSelectionScore(robotPose), 
                        a.getSelectionScore(robotPose)))
                .collect(Collectors.toList());
    }
    
    /**
     * Get POIs to avoid (obstacles and opponents)
     */
    public List<PointOfInterest> getPOIsToAvoid(Pose2d robotPose) {
        return activePOIs.stream()
                .filter(poi -> poi.getType().isObstacle() || poi.getType() == PointOfInterest.POIType.OPPONENT_ROBOT)
                .filter(poi -> poi.isWithinSafetyRange(robotPose))
                .sorted((a, b) -> Double.compare(a.distanceTo(robotPose), b.distanceTo(robotPose)))
                .collect(Collectors.toList());
    }
    
    /**
     * Get POIs to maintain distance from
     */
    public List<PointOfInterest> getPOIsToMaintainDistance(Pose2d robotPose) {
        return activePOIs.stream()
                .filter(PointOfInterest::shouldMaintainDistance)
                .filter(poi -> poi.distanceTo(robotPose) < poi.getRecommendedDistance())
                .collect(Collectors.toList());
    }
    
    // ========== NAVIGATION AND PATHFINDING ==========
    
    /**
     * Get the optimal pose to interact with a target POI
     */
    public Pose2d getOptimalInteractionPose(String poiId, Pose2d currentRobotPose) {
        PointOfInterest poi = allPOIs.get(poiId);
        if (poi != null) {
            return poi.getInteractionPose(currentRobotPose);
        }
        return currentRobotPose;
    }
    
    /**
     * Check if path from current position to target is clear of obstacles
     */
    public boolean isPathClear(Pose2d from, Translation2d to) {
        List<PointOfInterest> obstacles = getPOIsByType(PointOfInterest.POIType.OBSTACLE);
        
        for (PointOfInterest obstacle : obstacles) {
            if (!obstacle.isActive()) continue;
            
            // Simple line-circle intersection check
            if (isLineIntersectingCircle(from.getTranslation(), to, 
                    obstacle.getPosition(), obstacle.getSafetyRange())) {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Get intermediate waypoints to avoid obstacles
     */
    public List<Translation2d> getAvoidanceWaypoints(Pose2d from, Translation2d to) {
        List<Translation2d> waypoints = new ArrayList<>();
        
        // Simple obstacle avoidance - go around obstacles
        List<PointOfInterest> obstacles = getPOIsToAvoid(from);
        
        for (PointOfInterest obstacle : obstacles) {
            Translation2d avoidPos = obstacle.getPosition();
            double avoidDistance = obstacle.getRecommendedDistance();
            
            // Calculate perpendicular avoidance point
            Translation2d toObstacle = avoidPos.minus(from.getTranslation());
            Translation2d perpendicular = new Translation2d(-toObstacle.getY(), toObstacle.getX());
            perpendicular = perpendicular.div(perpendicular.getNorm()).times(avoidDistance);
            
            Translation2d waypoint = avoidPos.plus(perpendicular);
            waypoints.add(waypoint);
        }
        
        return waypoints;
    }
    
    // ========== POI QUERIES ==========
    
    /**
     * Get POI by ID
     */
    public PointOfInterest getPOI(String id) {
        return allPOIs.get(id);
    }
    
    /**
     * Get all POIs of a specific type
     */
    public List<PointOfInterest> getPOIsByType(PointOfInterest.POIType type) {
        return poisByType.getOrDefault(type, new ArrayList<>()).stream()
                .filter(PointOfInterest::isActive)
                .collect(Collectors.toList());
    }
    
    /**
     * Get all POIs within a certain range
     */
    public List<PointOfInterest> getPOIsInRange(Pose2d robotPose, double range) {
        return activePOIs.stream()
                .filter(poi -> poi.distanceTo(robotPose) <= range)
                .collect(Collectors.toList());
    }
    
    /**
     * Get the nearest POI of a specific type
     */
    public PointOfInterest getNearestPOI(Pose2d robotPose, PointOfInterest.POIType type) {
        return getPOIsByType(type).stream()
                .min((a, b) -> Double.compare(a.distanceTo(robotPose), b.distanceTo(robotPose)))
                .orElse(null);
    }
    
    /**
     * Get all active POIs
     */
    public List<PointOfInterest> getAllActivePOIs() {
        return new ArrayList<>(activePOIs);
    }
    
    /**
     * Get POIs by priority level
     */
    public List<PointOfInterest> getPOIsByPriority(PointOfInterest.POIPriority priority) {
        return activePOIs.stream()
                .filter(poi -> poi.getPriority() == priority)
                .collect(Collectors.toList());
    }
    
    // ========== UTILITY METHODS ==========
    
    /**
     * Update POI values based on game state
     */
    public void updatePOIValues(double timeRemaining, double allianceScore) {
        for (PointOfInterest poi : activePOIs) {
            double newValue = calculatePOIValue(poi, timeRemaining, allianceScore);
            poi.updateValue(newValue);
        }
    }
    
    /**
     * Calculate the strategic value of a POI
     */
    private double calculatePOIValue(PointOfInterest poi, double timeRemaining, double allianceScore) {
        double value = 0.0;
        
        switch (poi.getType()) {
            case SCORING_LOCATION:
                // Higher value when time is running out
                value = 10.0 * (1.0 - timeRemaining / 20.0);
                break;
            case FUEL_SOURCE:
                // Higher value when low on time but need fuel
                value = 8.0 * (timeRemaining / 20.0);
                break;
            case CLIMBING_ZONE:
                // High value at end of match
                if (timeRemaining < 5.0) {
                    value = 15.0;
                }
                break;
            case OBSTACLE:
                // Negative value (want to avoid)
                value = -5.0;
                break;
            case OPPONENT_ROBOT:
                // Negative value, but higher for closer opponents
                value = -3.0;
                break;
            default:
                value = 0.0;
                break;
        }
        
        return value;
    }
    
    /**
     * Clean up old POIs
     */
    public void cleanupOldPOIs(double maxAge) {
        Iterator<PointOfInterest> iterator = activePOIs.iterator();
        while (iterator.hasNext()) {
            PointOfInterest poi = iterator.next();
            if (!poi.isValid(maxAge)) {
                iterator.remove();
                allPOIs.remove(poi.getId());
            }
        }
        
        clearCache();
    }
    
    /**
     * Get field statistics
     */
    public FieldStatistics getFieldStatistics() {
        int totalPOIs = allPOIs.size();
        int activePOIsCount = activePOIs.size();
        int referencePOIs = getPOIsByType(PointOfInterest.POIType.APRIL_TAG).size();
        int targetPOIs = (int) activePOIs.stream().filter(poi -> poi.getType().isTarget()).count();
        int obstaclePOIs = (int) activePOIs.stream().filter(poi -> poi.getType().isObstacle()).count();
        
        return new FieldStatistics(totalPOIs, activePOIsCount, referencePOIs, targetPOIs, obstaclePOIs);
    }
    
    // ========== PRIVATE HELPER METHODS ==========
    
    private void updateCacheIfNeeded() {
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (currentTime - lastCacheUpdate > 0.1) { // Update cache every 100ms
            clearCache();
            lastCacheUpdate = currentTime;
        }
    }
    
    private void clearCache() {
        lastCacheUpdate = 0.0;
    }
    
    private boolean isLineIntersectingCircle(Translation2d lineStart, Translation2d lineEnd, 
                                        Translation2d circleCenter, double radius) {
        Translation2d d = lineEnd.minus(lineStart);
        Translation2d f = lineStart.minus(circleCenter);
        
        double a = d.dot(d);
        double b = 2 * f.dot(d);
        double c = f.dot(f) - radius * radius;
        
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return false;
        }
        
        discriminant = Math.sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);
        
        return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
    }
    
    /**
     * Field statistics container
     */
    public static class FieldStatistics {
        public final int totalPOIs;
        public final int activePOIs;
        public final int referencePOIs;
        public final int targetPOIs;
        public final int obstaclePOIs;
        
        public FieldStatistics(int totalPOIs, int activePOIs, int referencePOIs, 
                           int targetPOIs, int obstaclePOIs) {
            this.totalPOIs = totalPOIs;
            this.activePOIs = activePOIs;
            this.referencePOIs = referencePOIs;
            this.targetPOIs = targetPOIs;
            this.obstaclePOIs = obstaclePOIs;
        }
        
        @Override
        public String toString() {
            return String.format("FieldStats[Total: %d, Active: %d, Ref: %d, Targets: %d, Obstacles: %d]",
                    totalPOIs, activePOIs, referencePOIs, targetPOIs, obstaclePOIs);
        }
    }
}
