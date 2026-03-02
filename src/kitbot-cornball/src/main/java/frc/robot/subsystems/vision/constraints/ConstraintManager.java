package frc.robot.subsystems.vision.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.limelight.LimelightTarget;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Manages multiple constraints and combines their outputs.
 */
public class ConstraintManager {
    private final Map<String, AprilTagConstraint> constraints = new HashMap<>();
    private final List<AprilTagConstraint> activeConstraints = new ArrayList<>();

    /**
     * Adds a new constraint.
     * @param name Unique name for the constraint
     * @param constraint The constraint to add
     * @return true if added, false if a constraint with this name already exists
     */
    public boolean addConstraint(String name, AprilTagConstraint constraint) {
        if (constraints.containsKey(name)) {
            return false;
        }
        constraints.put(name, constraint);
        if (constraint.isActive()) {
            activeConstraints.add(constraint);
        }
        return true;
    }

    /**
     * Gets a constraint by name.
     * @param name Name of the constraint
     * @return The constraint, or empty if not found
     */
    public Optional<AprilTagConstraint> getConstraint(String name) {
        return Optional.ofNullable(constraints.get(name));
    }

    /**
     * Sets whether a constraint is active.
     * @param name Name of the constraint
     * @param active Whether the constraint should be active
     * @return true if the constraint was found and updated, false otherwise
     */
    public boolean setConstraintActive(String name, boolean active) {
        return getConstraint(name).map(constraint -> {
            if (active && !constraint.isActive()) {
                activeConstraints.add(constraint);
            } else if (!active && constraint.isActive()) {
                activeConstraints.remove(constraint);
            }
            constraint.setActive(active);
            return true;
        }).orElse(false);
    }

    /**
     * Combines the outputs of all active constraints.
     * @param target The current AprilTag target
     * @param currentPose Current robot pose (if available)
     * @return Combined chassis speeds from all active constraints
     */
    public ChassisSpeeds calculate(LimelightTarget target, Optional<Pose2d> currentPose) {
        double vx = 0, vy = 0, omega = 0;

        for (AprilTagConstraint constraint : activeConstraints) {
            Optional<ChassisSpeeds> speeds = constraint.calculate(target, currentPose);
            if (speeds.isPresent()) {
                vx += speeds.get().vxMetersPerSecond;
                vy += speeds.get().vyMetersPerSecond;
                omega += speeds.get().omegaRadiansPerSecond;
            }
        }

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * @return List of all registered constraint names
     */
    public List<String> getConstraintNames() {
        return new ArrayList<>(constraints.keySet());
    }

    /**
     * @return List of all registered constraints
     */
    public List<AprilTagConstraint> getConstraints() {
        return new ArrayList<>(constraints.values());
    }

    /**
     * @return List of all active constraints
     */
    public List<AprilTagConstraint> getActiveConstraints() {
        return new ArrayList<>(activeConstraints);
    }
}
