package frc.robot.subsystems.vision.constraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

/**
 * Constraint to maintain a specific distance from the AprilTag.
 */
public class DistanceConstraint implements AprilTagConstraint {
    private final PIDController distanceController;
    private double targetDistanceMeters;
    private boolean isActive;
    private final String name;

    /**
     * Creates a new DistanceConstraint
     * @param name Display name for this constraint
     * @param targetDistanceMeters Desired distance from the target in meters
     * @param kP Proportional gain for distance control
     * @param kI Integral gain for distance control
     * @param kD Derivative gain for distance control
     */
    public DistanceConstraint(String name, double targetDistanceMeters, double kP, double kI, double kD) {
        this.name = name;
        this.distanceController = new PIDController(kP, kI, kD);
        this.targetDistanceMeters = targetDistanceMeters;
        this.isActive = true;
        
        distanceController.setTolerance(0.05); // 5cm tolerance
    }

    @Override
    public Optional<ChassisSpeeds> calculate(PhotonTrackedTarget target, Optional<Pose2d> currentPose) {
        if (!isActive) {
            return Optional.empty();
        }

        // Get distance to target from the camera's perspective
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        double speed = -distanceController.calculate(distance, targetDistanceMeters);
        
        return Optional.of(new ChassisSpeeds(speed, 0, 0));
    }

    public void setTargetDistance(double distanceMeters) {
        this.targetDistanceMeters = distanceMeters;
    }

    public double getTargetDistance() {
        return targetDistanceMeters;
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
