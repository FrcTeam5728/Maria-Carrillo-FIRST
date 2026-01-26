package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class FieldConstants {
    // Field dimensions in inches (convert to meters when needed)
    public static final double FIELD_LENGTH_INCHES = 648.0;
    public static final double FIELD_WIDTH_INCHES = 324.0;
    
    // Hub positions (in inches, from the field coordinate system)
    public static final class HubPositions {
        // Red Hub (average position of red hub tags)
        public static final Translation3d RED_HUB = new Translation3d(
            (469.11 + 445.35) / 2.0 * 0.0254,  // Average x position in meters
            (182.60 + 135.09) / 2.0 * 0.0254,  // Average y position in meters
            44.25 * 0.0254                      // Height in meters
        );
        
        // Blue Hub (assuming symmetric to red hub)
        public static final Translation3d BLUE_HUB = new Translation3d(
            FIELD_LENGTH_INCHES * 0.0254 - RED_HUB.getX(),  // Mirror across field length
            FIELD_WIDTH_INCHES * 0.0254 - RED_HUB.getY(),   // Mirror across field width
            RED_HUB.getZ()                                  // Same height
        );
        
        // Hub target height (distance from floor to center of the target area)
        public static final double TARGET_HEIGHT_METERS = 1.96;  // ~6.43 feet (adjust based on actual game manual)
    }
    
    // AprilTag IDs for hubs (from apriltag_transforms.json)
    public static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5};
    // Blue hub tags would be the same IDs but on the blue side
    
    private FieldConstants() {
        // Private constructor to prevent instantiation
    }
    
    /**
     * Converts inches to meters
     */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    
    /**
     * Converts meters to inches
     */
    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }
}
