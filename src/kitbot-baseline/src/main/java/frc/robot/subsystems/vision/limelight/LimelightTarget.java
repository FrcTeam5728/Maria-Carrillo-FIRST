package frc.robot.subsystems.vision.limelight;

/**
 * Represents a target detected by the Limelight camera system.
 * This class encapsulates the target data from NetworkTable values.
 */
public class LimelightTarget {
    private final boolean hasTarget;
    private final double tx;  // Horizontal offset from crosshair to target (-27 to 27 degrees)
    private final double ty;  // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
    private final double ta;  // Target area (0% to 100% of image)
    private final int tid;   // ID of the primary in-view AprilTag
    private final double distanceMeters;
    
    /**
     * Creates a new LimelightTarget with the specified values.
     * 
     * @param hasTarget Whether the limelight has any valid targets (0 or 1)
     * @param tx Horizontal offset from crosshair to target in degrees
     * @param ty Vertical offset from crosshair to target in degrees
     * @param ta Target area as percentage of image
     * @param tid ID of the AprilTag
     * @param distanceMeters Calculated distance to the target in meters
     */
    public LimelightTarget(boolean hasTarget, double tx, double ty, double ta, int tid, double distanceMeters) {
        this.hasTarget = hasTarget;
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
        this.tid = tid;
        this.distanceMeters = distanceMeters;
    }
    
    /**
     * @return True if the limelight has a valid target, false otherwise
     */
    public boolean hasTarget() {
        return hasTarget;
    }
    
    /**
     * @return Horizontal offset from crosshair to target in degrees (-27 to 27)
     */
    public double getTx() {
        return tx;
    }
    
    /**
     * @return Vertical offset from crosshair to target in degrees (-20.5 to 20.5)
     */
    public double getTy() {
        return ty;
    }
    
    /**
     * @return Target area as percentage of image (0% to 100%)
     */
    public double getTa() {
        return ta;
    }
    
    /**
     * @return ID of the primary in-view AprilTag
     */
    public int getTid() {
        return tid;
    }
    
    /**
     * @return Calculated distance to the target in meters
     */
    public double getDistanceMeters() {
        return distanceMeters;
    }
    
    /**
     * @return Horizontal angle to the target in degrees (same as tx)
     */
    public double getYawDegrees() {
        return tx;
    }
    
    /**
     * @return Estimated X position relative to target (simplified calculation)
     */
    public double getX() {
        // Simplified calculation based on horizontal angle and distance
        // This is an approximation - for more accurate results, use 3D pose estimation
        return distanceMeters * Math.sin(Math.toRadians(tx));
    }
    
    /**
     * @return Estimated Y position relative to target (simplified calculation)
     */
    public double getY() {
        // Simplified calculation based on vertical angle and distance
        // This is an approximation - for more accurate results, use 3D pose estimation
        return distanceMeters * Math.cos(Math.toRadians(ty));
    }
    
    /**
     * Creates a LimelightTarget representing no target detected.
     * 
     * @return A LimelightTarget with hasTarget = false
     */
    public static LimelightTarget empty() {
        return new LimelightTarget(false, 0, 0, 0, -1, 0);
    }
}
