package frc.robot.field.interaction;

import frc.robot.field.core.PointOfInterest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;

/**
 * POI Interaction system for robot positioning relative to Points of Interest
 * Handles maintaining distance, rotation, and interaction completion
 */
public class POIInteraction {
    
    /**
     * Container for POI interaction strategy
     */
    public static class Strategy {
        private final PointOfInterest target;
        private final double targetDistance;
        private final Rotation2d targetHeading;
        private final double distanceError;
        private final double headingError;
        
        public Strategy(PointOfInterest target, double targetDistance, Rotation2d targetHeading, 
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
     * Calculate POI interaction strategy for maintaining distance and rotation
     * @param robotPose Current robot pose
     * @param targetPOI Target point of interest
     * @param desiredDistance Desired distance to maintain from POI
     * @return Interaction strategy with distance, heading, and error corrections
     */
    public static Strategy calculateStrategy(Pose2d robotPose, PointOfInterest targetPOI, double desiredDistance) {
        if (targetPOI == null || !targetPOI.isActive()) {
            return new Strategy(null, 0.0, robotPose.getRotation(), 0.0, 0.0);
        }
        
        // Calculate current distance to POI
        double currentDistance = targetPOI.distanceTo(robotPose);
        
        // Calculate desired position (maintain distance from POI)
        // Note: desiredPosition calculated but not used directly - distance/heading errors provide control
        
        // Calculate heading to face the POI
        Translation2d toPOI = targetPOI.getPosition().minus(robotPose.getTranslation());
        Rotation2d desiredHeading = new Rotation2d(Math.atan2(toPOI.getY(), toPOI.getX()));
        
        // Calculate errors for correction
        double distanceError = currentDistance - desiredDistance;
        double headingError = desiredHeading.minus(robotPose.getRotation()).getRadians();
        
        // Normalize heading error to [-π, π]
        headingError = MathUtil.inputModulus(headingError, -Math.PI, Math.PI);
        
        return new Strategy(targetPOI, desiredDistance, desiredHeading, distanceError, headingError);
    }
    
    /**
     * Calculate drive commands for POI interaction
     * @param strategy POI interaction strategy
     * @param maxSpeed Maximum drive speed
     * @param maxTurnRate Maximum turn rate
     * @return Drive commands [forwardSpeed, turnSpeed]
     */
    public static double[] calculateDriveCommands(Strategy strategy, double maxSpeed, double maxTurnRate) {
        if (!strategy.hasTarget()) {
            return new double[]{0.0, 0.0}; // No target, stop
        }
        
        // Distance control (proportional)
        double distanceCorrection = strategy.getDistanceError() * 0.5; // P gain for distance
        distanceCorrection = Math.max(-maxSpeed, Math.min(maxSpeed, distanceCorrection));
        
        // Heading control (proportional)
        double headingCorrection = strategy.getHeadingError() * 2.0; // P gain for heading
        headingCorrection = Math.max(-maxTurnRate, Math.min(maxTurnRate, headingCorrection));
        
        // Combine into drive commands
        double forwardSpeed = distanceCorrection;
        double turnSpeed = headingCorrection;
        
        return new double[]{forwardSpeed, turnSpeed};
    }
    
    /**
     * Check if robot is in acceptable POI interaction range
     * @param strategy POI interaction strategy
     * @param distanceTolerance Acceptable distance tolerance
     * @param headingTolerance Acceptable heading tolerance (radians)
     * @return True if within acceptable range
     */
    public static boolean isInteractionComplete(Strategy strategy, double distanceTolerance, double headingTolerance) {
        if (!strategy.hasTarget()) {
            return false; // No target to interact with
        }
        
        double distanceError = Math.abs(strategy.getDistanceError());
        double headingError = Math.abs(strategy.getHeadingError());
        
        return distanceError <= distanceTolerance && headingError <= headingTolerance;
    }
    
    /**
     * Container for interaction actions
     */
    public static enum InteractionAction {
        NONE("None"),
        MAINTAIN_DISTANCE("Maintain Distance"),
        ROTATE_TO_TARGET("Rotate to Target"),
        SHOOT("Shoot"),
        COLLECT("Collect"),
        WAIT("Wait"),
        SCAN("Scan"),
        DEPLOY("Deploy"),
        RETREAT("Retreat");
        
        private final String displayName;
        
        InteractionAction(String displayName) {
            this.displayName = displayName;
        }
        
        public String getDisplayName() { return displayName; }
        
        @Override
        public String toString() { return displayName; }
    }
}
