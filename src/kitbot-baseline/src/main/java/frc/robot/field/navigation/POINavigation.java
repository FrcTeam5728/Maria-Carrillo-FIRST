package frc.robot.field.navigation;

import frc.robot.field.core.PointOfInterest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.ArrayList;

/**
 * Path planning and navigation system for Points of Interest
 * Handles optimal routing and waypoint generation
 */
public class POINavigation {
    
    /**
     * Container for a planned path through POIs
     */
    public static class Path {
        private final Pose2d startPose;
        private final Translation2d[] waypoints;
        private final double totalDistance;
        private final List<PointOfInterest> poiSequence;
        
        public Path(Pose2d startPose, Translation2d[] waypoints, double totalDistance, List<PointOfInterest> poiSequence) {
            this.startPose = startPose;
            this.waypoints = waypoints;
            this.totalDistance = totalDistance;
            this.poiSequence = new ArrayList<>(poiSequence);
        }
        
        public Pose2d getStartPose() { return startPose; }
        public Translation2d[] getWaypoints() { return waypoints; }
        public double getTotalDistance() { return totalDistance; }
        public List<PointOfInterest> getPOISequence() { return new ArrayList<>(poiSequence); }
        public int getWaypointCount() { return waypoints.length; }
        
        @Override
        public String toString() {
            return String.format("Path[waypoints=%d, distance=%.2f, pois=%d]", 
                    waypoints.length, totalDistance, poiSequence.size());
        }
    }
    
    /**
     * Calculate optimal path through multiple POIs using nearest-neighbor algorithm
     * @param startPose Starting robot pose
     * @param pois List of POIs to visit
     * @return Optimal path through POIs
     */
    public static Path calculateOptimalPath(Pose2d startPose, List<PointOfInterest> pois) {
        if (pois.isEmpty()) {
            return new Path(startPose, new Translation2d[0], 0.0, new ArrayList<>());
        }
        
        // Simple nearest-neighbor path planning
        List<PointOfInterest> unvisited = new ArrayList<>(pois);
        List<PointOfInterest> sequence = new ArrayList<>();
        List<Translation2d> waypoints = new ArrayList<>();
        
        Pose2d currentPose = startPose;
        double totalDistance = 0.0;
        
        while (!unvisited.isEmpty()) {
            // Find nearest unvisited POI
            PointOfInterest nearest = findNearestPOI(currentPose, unvisited);
            if (nearest == null) break;
            
            sequence.add(nearest);
            waypoints.add(nearest.getPosition());
            totalDistance += nearest.distanceTo(currentPose);
            currentPose = nearest.getInteractionPose(currentPose);
            unvisited.remove(nearest);
        }
        
        return new Path(startPose, waypoints.toArray(new Translation2d[0]), totalDistance, sequence);
    }
    
    /**
     * Find the nearest POI to current pose
     * @param pose Current robot pose
     * @param pois List of POIs to search
     * @return Nearest POI or null if list is empty
     */
    public static PointOfInterest findNearestPOI(Pose2d pose, List<PointOfInterest> pois) {
        PointOfInterest nearest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (PointOfInterest poi : pois) {
            if (!poi.isActive()) continue;
            
            double distance = poi.distanceTo(pose);
            if (distance < minDistance) {
                minDistance = distance;
                nearest = poi;
            }
        }
        
        return nearest;
    }
    
    /**
     * Calculate path to avoid obstacles
     * @param startPose Starting pose
     * @param targetPose Target pose
     * @param obstacles List of obstacle POIs
     * @return Path that avoids obstacles
     */
    public static Path calculateAvoidancePath(Pose2d startPose, Pose2d targetPose, List<PointOfInterest> obstacles) {
        if (obstacles.isEmpty()) {
            // Direct path if no obstacles
            return new Path(startPose, new Translation2d[]{targetPose.getTranslation()}, 
                        startPose.getTranslation().getDistance(targetPose.getTranslation()), 
                        new ArrayList<>());
        }
        
        // Simple obstacle avoidance: add intermediate waypoints
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(startPose.getTranslation());
        
        // Add waypoints to avoid obstacles
        for (PointOfInterest obstacle : obstacles) {
            if (!obstacle.isActive()) continue;
            
            // Check if direct path intersects obstacle
            Translation2d directPath = targetPose.getTranslation().minus(startPose.getTranslation());
            Translation2d toObstacle = obstacle.getPosition().minus(startPose.getTranslation());
            
            // Simple check: if obstacle is close to direct path, add waypoint around it
            double projection = (toObstacle.getX() * directPath.getX() + toObstacle.getY() * directPath.getY()) / 
                           (directPath.getX() * directPath.getX() + directPath.getY() * directPath.getY());
            
            if (projection > 0 && projection < 1 && toObstacle.getNorm() < obstacle.getRecommendedDistance()) {
                // Add waypoint around obstacle
                Translation2d avoidance = new Translation2d(-directPath.getY(), directPath.getX());
                avoidance = avoidance.div(avoidance.getNorm()).times(obstacle.getRecommendedDistance() * 1.5);
                waypoints.add(obstacle.getPosition().plus(avoidance));
            }
        }
        
        waypoints.add(targetPose.getTranslation());
        
        return new Path(startPose, waypoints.toArray(new Translation2d[0]), 
                    calculatePathDistance(waypoints), new ArrayList<>());
    }
    
    /**
     * Calculate total distance of a path
     * @param waypoints List of waypoints
     * @return Total path distance
     */
    private static double calculatePathDistance(List<Translation2d> waypoints) {
        if (waypoints.size() < 2) return 0.0;
        
        double total = 0.0;
        for (int i = 1; i < waypoints.size(); i++) {
            total += waypoints.get(i).getDistance(waypoints.get(i - 1));
        }
        return total;
    }
}
