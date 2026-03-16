package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility to check available field layouts and shooting target information.
 * Provides information about 2026 field layout and AprilTag positions.
 */
public class FieldLayoutChecker {
    
    /**
     * Checks if 2026 field layout is available.
     * 
     * @return True if 2026 field is available
     */
    public static boolean has2026Field() {
        try {
            // Try to load 2026 field layout
            AprilTagFields.k2026Field.loadAprilTagLayoutField();
            return true;
        } catch (Exception e) {
            System.out.println("2026 field layout not available: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Gets information about available field layouts.
     */
    public static void printFieldInfo() {
        System.out.println("=== FIELD LAYOUT INFORMATION ===");
        
        // Check for 2026 field
        if (has2026Field()) {
            System.out.println("✓ 2026 Field Layout Available");
            print2026FieldInfo();
        } else {
            System.out.println("✗ 2026 Field Layout Not Available");
            System.out.println("Using default field layout instead");
            printDefaultFieldInfo();
        }
        
        System.out.println("=============================");
    }
    
    /**
     * Prints specific 2026 field information.
     */
    private static void print2026FieldInfo() {
        try {
            AprilTagFieldLayout field2026 = AprilTagFields.k2026Field.loadAprilTagLayoutField();
            
            System.out.println("2026 Field Layout:");
            System.out.println("  Total AprilTags: " + field2026.getTags().size());
            
            // Look for common shooting target tags
            int[] shootingTargetTags = {1, 2, 3, 4, 5, 6, 7, 8, 9}; // Common speaker/amp tags
            
            System.out.println("  Shooting Target AprilTags:");
            for (int tagId : shootingTargetTags) {
                var tagPose = field2026.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    var pose = tagPose.get();
                    System.out.println(String.format("    Tag %d: (%.2f, %.2f) rotation %.1f°", 
                        tagId, pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
                } else {
                    System.out.println("    Tag " + tagId + ": Not found in 2026 layout");
                }
            }
            
        } catch (Exception e) {
            System.err.println("Error accessing 2026 field info: " + e.getMessage());
        }
    }
    
    /**
     * Prints default field information.
     */
    private static void printDefaultFieldInfo() {
        try {
            AprilTagFieldLayout defaultField = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            
            System.out.println("Default Field Layout:");
            System.out.println("  Total AprilTags: " + defaultField.getTags().size());
            
            // Look for common shooting target tags
            int[] shootingTargetTags = {1, 2, 3, 4, 5, 6, 7, 8, 9};
            
            System.out.println("  Available Shooting Target AprilTags:");
            for (int tagId : shootingTargetTags) {
                var tagPose = defaultField.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    var pose = tagPose.get();
                    System.out.println(String.format("    Tag %d: (%.2f, %.2f) rotation %.1f°", 
                        tagId, pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
                } else {
                    System.out.println("    Tag " + tagId + ": Not found in default layout");
                }
            }
            
        } catch (Exception e) {
            System.err.println("Error accessing default field info: " + e.getMessage());
        }
    }
    
    /**
     * Gets shooting target information for a specific tag ID.
     * 
     * @param tagId The AprilTag ID to check
     * @return Information about the shooting target
     */
    public static String getShootingTargetInfo(int tagId) {
        try {
            AprilTagFieldLayout field = has2026Field() ? 
                AprilTagFields.k2026Field.loadAprilTagLayoutField() : 
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            
            var tagPose = field.getTagPose(tagId);
            if (tagPose.isPresent()) {
                var pose = tagPose.get();
                return String.format("Tag %d at (%.2f, %.2f) facing %.1f°", 
                    tagId, pose.getX(), pose.getY(), pose.getRotation().getDegrees());
            } else {
                return "Tag " + tagId + " not found in field layout";
            }
        } catch (Exception e) {
            return "Error getting tag info: " + e.getMessage();
        }
    }
    
    /**
     * Checks if a tag ID is a known shooting target.
     * 
     * @param tagId The tag ID to check
     * @return True if this is a shooting target tag
     */
    public static boolean isShootingTarget(int tagId) {
        // Common 2026 shooting target tags (speaker, amp, etc.)
        int[] shootingTargets = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
        
        for (int targetId : shootingTargets) {
            if (tagId == targetId) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Gets all available shooting target tag IDs.
     * 
     * @return Array of shooting target tag IDs
     */
    public static int[] getShootingTargetTags() {
        return new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    }
}
