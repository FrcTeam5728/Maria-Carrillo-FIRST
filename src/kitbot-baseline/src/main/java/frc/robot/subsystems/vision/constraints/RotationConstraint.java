package frc.robot.subsystems.vision.constraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

/**
 * Constraint to face towards the AprilTag.
 */
public class RotationConstraint implements AprilTagConstraint {
    private final PIDController rotationController;
    private boolean isActive;
    private final String name;

    /**
     * Creates a new RotationConstraint
     * @param name Display name for this constraint
     * @param kP Proportional gain for rotation control
     * @param kI Integral gain for rotation control
     * @param kD Derivative gain for rotation control
     */
    public RotationConstraint(String name, double kP, double kI, double kD) {
        this.name = name;
        this.rotationController = new PIDController(kP, kI, kD);
        this.isActive = true;
        
        rotationController.setTolerance(2.0); // 2 degrees tolerance
        rotationController.enableContinuousInput(-180, 180);
    }

    @Override
    public Optional<ChassisSpeeds> calculate(PhotonTrackedTarget target, Optional<Pose2d> currentPose) {
        if (!isActive) {
            return Optional.empty();
        }

        // Get the yaw (horizontal angle) to the target
        double yaw = target.getYaw();
        double rotation = rotationController.calculate(yaw, 0);
        
        return Optional.of(new ChassisSpeeds(0, 0, rotation));
    }

    @Override
    public boolean isActive() {
        return isActive;
    }

    @Override
    public void setActive(boolean active) {
        this.isActive = active;
    }

    @Override
    public String getName() {
        return name;
    }
}
