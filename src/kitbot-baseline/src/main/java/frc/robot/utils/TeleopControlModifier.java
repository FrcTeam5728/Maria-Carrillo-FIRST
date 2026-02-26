package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.constraints.ConstraintManager;
import frc.robot.Constants.DriveConstants;

/**
 * A utility class for modifying teleop control inputs with constraints.
 * This class processes raw joystick inputs and applies constraints before they reach the drive subsystem.
 */
public class TeleopControlModifier {
    private final AprilTagSubsystem aprilTagSubsystem;
    private boolean useConstraints = false;
    private double lastForward = 0;
    private double lastRotation = 0;
    private static final double DEADBAND = 0.05;

    /**
     * Creates a new TeleopControlModifier with the specified AprilTagSubsystem.
     * 
     * @param aprilTagSubsystem The AprilTag subsystem to use for vision-based constraints
     */
    public TeleopControlModifier(AprilTagSubsystem aprilTagSubsystem) {
        this.aprilTagSubsystem = aprilTagSubsystem;
    }

    /**
     * Enables or disables the use of constraints.
     * 
     * @param enabled Whether to enable constraints
     */
    public void setConstraintsEnabled(boolean enabled) {
        this.useConstraints = enabled;
    }

    /**
     * Applies constraints to the control inputs.
     * 
     * @param forward The forward/backward input (-1.0 to 1.0)
     * @param rotation The rotation input (-1.0 to 1.0)
     * @return Modified control inputs with constraints applied
     */
    public double[] applyConstraints(double forward, double rotation) {
        // Apply deadband
        forward = applyDeadband(forward, DEADBAND);
        rotation = applyDeadband(rotation, DEADBAND);

        // If constraints are disabled, return raw values
        if (!useConstraints || aprilTagSubsystem == null) {
            lastForward = forward;
            lastRotation = rotation;
            return new double[]{forward, rotation};
        }

        // If we have a target, apply vision-based constraints
        if (aprilTagSubsystem.hasTarget()) {
            // Get the constrained speeds from the AprilTag subsystem
            var targetSpeeds = aprilTagSubsystem.getTargetSpeeds();
            if (targetSpeeds.isPresent()) {
                ChassisSpeeds speeds = targetSpeeds.get();
                // Blend between driver input and constrained speeds
                // The blend factor could be made configurable if needed
                double blendFactor = 0.5;
                double constrainedForward = speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
                double constrainedRotation = speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeedRadiansPerSecond;
                
                forward = lerp(forward, constrainedForward, blendFactor);
                rotation = lerp(rotation, constrainedRotation, blendFactor);
            }
        }

        // Apply rate limiting to prevent sudden changes
        forward = rateLimit(forward, lastForward, 0.1);
        rotation = rateLimit(rotation, lastRotation, 0.1);

        lastForward = forward;
        lastRotation = rotation;

        return new double[]{forward, rotation};
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    private double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    private double rateLimit(double newValue, double lastValue, double maxChange) {
        if (newValue > lastValue + maxChange) {
            return lastValue + maxChange;
        }
        if (newValue < lastValue - maxChange) {
            return lastValue - maxChange;
        }
        return newValue;
    }
}
