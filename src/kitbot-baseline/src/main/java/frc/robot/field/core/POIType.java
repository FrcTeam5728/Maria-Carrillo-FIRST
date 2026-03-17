package frc.robot.field.core;

/**
 * Enumeration of Point of Interest types
 * Categorizes different types of field locations
 */
public enum POIType {
    // Scoring locations
    HUB("Hub", true, 2.0),
    
    // Game pieces
    FUEL("Fuel", false, 0.5),
    
    // Field elements
    STAGE("Stage", false, 0.0),
    CHARGING_STATION("Charging Station", false, 0.0),
    BUMPER("Bumper", false, 0.0),
    HUB_FUNNEL("Hub Funnel", false, 0.0),
    NEUTRAL_ZONE("Neutral Zone", false, 0.0),
    DEPOT("Depot", false, 0.0),
    TRENCH("Trench", false, 0.0),
    
    // Strategic locations
    WAYPOINT("Waypoint", false, 0.0),
    DEFENSIVE_POSITION("Defensive Position", false, 0.0),
    
    // Vision/Navigation
    APRIL_TAG("AprilTag", false, 0.0),
    
    // Obstacles
    ROBOT("Robot", false, 0.0),
    OBSTACLE("Obstacle", false, 0.0);
    
    private final String displayName;
    private final boolean isTarget;
    private final double baseValue;
    
    POIType(String displayName, boolean isTarget, double baseValue) {
        this.displayName = displayName;
        this.isTarget = isTarget;
        this.baseValue = baseValue;
    }
    
    public String getDisplayName() { return displayName; }
    public boolean isTarget() { return isTarget; }
    public double getBaseValue() { return baseValue; }
    
    @Override
    public String toString() { return displayName; }
}
