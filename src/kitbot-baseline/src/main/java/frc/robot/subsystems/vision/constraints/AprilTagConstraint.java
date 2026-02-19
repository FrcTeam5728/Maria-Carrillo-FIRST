package frc.robot.subsystems.vision.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

/**
 * Interface for defining constraints when tracking AprilTags.
 * Each constraint can modify the desired robot movement based on the target's state.
 */
public interface AprilTagConstraint {
    /**
     * Calculates the desired movement to satisfy this constraint.
     * @param target The current AprilTag target
     * @param currentPose Current robot pose (if available)
     * @return Optional ChassisSpeeds representing the desired movement, or empty if no movement needed
     */
    Optional<ChassisSpeeds> calculate(PhotonTrackedTarget target, Optional<Pose2d> currentPose);

    /**
     * @return True if this constraint is currently active
     */
    boolean isActive();

    /**
     * Sets whether this constraint is active
     */
    void setActive(boolean active);

    /**
     * @return A user-friendly name for this constraint
     */
    String getName();
}
